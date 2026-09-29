# jezero-lab — plan

**Goal:** train an RL policy that drives the JPL Open Source Rover v4 across a
simulated patch of Jezero Crater to a waypoint. Everything runs locally in Podman
on this machine (Fedora 44, 12 cores, 62 GB RAM, RX 7600): no cloud, no NVIDIA
dependency.

**Target stack:** Ubuntu 24.04 container → ROS 2 Jazzy + Gazebo Harmonic →
Gymnasium + Stable-Baselines3 (PyTorch, CPU) → TensorBoard.

**Starting constraint:** upstream `osr_gazebo` (in `nasa-jpl/osr-rover-code`) is
still **Gazebo Classic on ROS 2 Humble**. Classic went end-of-life in January 2025,
and "migrate to Harmonic / `ros_gz` for Jazzy" is an open follow-up in its README.
Phase 0 runs it as-is; Phase 1 ports it.

Lineage: successor to the 2020 AWS-JPL OSR Challenge work
(`joebarbere/AWS-JPL-OSR-Challenge`, branch `docker-cuda`). AWS RoboMaker, which
that challenge ran on, was discontinued on 2025-09-10.

---

## Phase 0 — Baseline on the upstream stack (~1 evening)

- Humble + Gazebo Classic container; run `osr_gazebo` exactly as published
  (`empty_world.launch.py` + teleop).
- GUI through X11/Wayland with `--device /dev/dri`, so rendering uses the AMD GPU
  via Mesa.
- **Done when:** you can drive the OSR with the keyboard. That is the known-good
  reference before anything changes.

### Phase 0 results — 2026-09-28

Upstream pinned at `6b17c22` (master, 2026-09-15; `v4.1.0-92`). Image
`jezero-lab:humble-classic`, 4.35 GB.

| Check | Result |
|---|---|
| Build | 4/4 packages (`osr_interfaces`, `osr_control`, `osr_bringup`, `osr_gazebo`) |
| Controllers | `joint_state_broadcaster`, `servo_controller`, `wheel_controller` all active |
| Rendering | hardware: `GFX1102` (RX 7600), Mesa 23.2.1, OpenGL 4.6 |
| Forward | 0.322 m/s for 0.3 commanded, measured against **sim** time |
| Rotate in place | `angular.y: 0.5` → ~26° yaw, ~5 cm drift |
| **Real-time factor** | **0.25**, same with and without GUI; `gzserver` pins one core |

**The RTF is the headline.** It isn't rendering; it's physics. Every `<collision>`
in `osr.urdf.xacro` is a full-resolution STL (44 MB total; `body.stl` alone is
11 MB, ~220k triangles). At 0.25× a single env gives RL a quarter of a
second of experience per second — Phase 4 would be unworkable as-is.

Baseline numbers for Phase 1 to match: **0.32 m/s at 0.3 commanded**, rotate-in-place
yaw sign negative for positive `angular.y`.

## Phase 1 — Port to Jazzy + Harmonic

- `gazebo_ros2_control` → `gz_ros2_control`; move the IMU plugin to its Harmonic
  equivalent; `spawn_entity.py` → `ros_gz_sim create`.
- Keep upstream's design where the sim reuses the rover's own `kinematics.py`, so
  sim and hardware share the same math.
- **Replace mesh collisions with primitives** (cylinders for wheels, boxes for body
  and links), keeping STLs as `<visual>` only. Phase 0 measured RTF 0.25 with
  mesh collisions. Target: RTF ≥ 5 headless — measure before and after, on Classic
  first if that's quicker, so the gain isn't confused with the Harmonic port.
- **Done when:** teleop behaves the same as Phase 0, including rotate-in-place
  (`angular.y`).
- Optional: open a pull request upstream, since it's their open follow-up.

## Phase 2 — Jezero world

- Download a HiRISE DTM of the Jezero delta; crop to a drivable patch of a few
  hundred metres.
- Load it as a Gazebo DEM heightmap; set gravity to 3.721 m/s².
- 3–5 waypoints loosely following Perseverance's route up the delta.
- **Done when:** the rover spawns on the terrain and drives without sinking through
  it or bouncing off.

## Phase 3 — Gym environment

`JezeroEnv(gymnasium.Env)`:

- **Observation:** pose, vector to goal, IMU, a small patch of terrain height samples
  around the rover. No camera at first; rendering cameras is where CPU-only training
  slows down.
- **Action:** continuous `[linear, angular]`, turned into wheel speeds and steering
  angles by `kinematics.py`.
- **Reward:** port of the 2020 reward: keep `at_destination()` with its bounding box,
  add penalties for tipping and collisions, and a step budget standing in for the
  power supply.
- **Reset:** full world and joint reset. Joint state not resetting cleanly was the
  2020 bug, so test reset first.
- Training code controls stepping (fixed physics steps per action) instead of the
  sim running in real time. **Spike early:** measure the gz Python API against a ROS
  bridge for step speed; keep ROS out of the training loop if the bridge is slow.
- **Done when:** a random agent runs 1,000 episodes without the sim hanging or
  leaking memory.

## Phase 4 — Training

- Parallel envs as separate sim processes, isolated with `GZ_PARTITION` +
  `ROS_DOMAIN_ID`. Start with 6–8, then tune based on CPU use.
- PPO first, then SAC. TensorBoard (replaces the 2020 ELK stack).
- Curriculum: flat ground, then the delta slope, then boulder fields.
- **Done when:** the policy beats a hand-written go-to-goal controller on success
  rate across held-out spawn points.

## Phase 5 — Evaluation and demo

- Eval script: fixed seeds, recorded videos, success rate and path-efficiency plots.
- Optional: camera observations, ROCm PyTorch on the RX 7600, sim-to-real prep for a
  physical OSR build.

---

## Repo layout

```
jezero-lab/
├── containers/       # Containerfile(s), GUI/run scripts
├── ros_ws/src/       # osr-rover-code (git submodule, pinned) + jezero_world pkg
├── terrain/          # DTM fetch + crop + heightmap conversion scripts
├── jezero_env/       # Gymnasium env, reward, reset logic
├── train/            # SB3 configs, launch N sims, TensorBoard logs → runs/
└── eval/             # eval + video recording
```

## Risks to watch

- **Sim speed on CPU.** Rocker-bogie physics on bumpy terrain might not run much
  faster than real time. Measure in Phase 3 before building on it; fall back to
  simpler terrain for early training.
- **The port may take longer than a weekend,** mostly because of controller setup in
  `gz_ros2_control`. Phase 0 exists so there's always something working to fall back to.
- **HiRISE files are large.** Crop hard; a first policy doesn't need cm resolution.
- **Disk:** `/home` was at 94% (59 GB free) on 2026-09-28. Each ROS desktop image is
  several GB; prune old images between phases.
