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

### Phase 1 results — 2026-09-29

New package `ros_ws/src/osr_gz` (derived from upstream `osr_gazebo`, Apache-2.0),
built in both `jezero-lab:humble-classic` and `jezero-lab:jazzy-harmonic`:

```
ros2 launch osr_gz sim.launch.py [sim:=classic|harmonic] [collision:=primitive|mesh]
                                 [gui:=true|false] [realtime:=true|false [step:=0.005]]
```

Same link/joint names and inertials as upstream, the same `osr_control` rover
node + `kinematics.py`, and upstream's command adapter unchanged. Measured with
`containers/bench.sh` (`tools/bench.py`): forward speed and rotate-in-place
against sim time, heading from the IMU.

| Config | RTF | Forward (0.3 cmd) | Rotate, 4 sim-s |
|---|---|---|---|
| Phase 0: upstream, Classic, mesh | 0.24 | 0.325 | −84° |
| Classic, `osr_gz` mesh (parity check) | 0.26 | 0.322 | −83° |
| Classic, primitive, real time | 0.98 (capped) | 0.326 | −103° |
| **Harmonic, primitive, real time** | 1.00 (capped) | 0.328 | −116° |
| Harmonic, mesh, real time | 0.99 (capped) | 0.327 | −115° |
| Harmonic, primitive, unthrottled, 1 ms | 3.1 | 0.324 | −115° |
| Harmonic, primitive, unthrottled, 2 / 4 / 8 ms | 6.2 / 11.7 / 22.4 | 0.327–0.329 | −119° / −124° / −130° |
| **Harmonic, primitive, unthrottled, 5 ms (default)** | **~16** (gz `/stats`, idle/driving/rotating 16.3/15.6/16.6) | 0.327 | −124° |

Findings, in order of how much they matter:

- **Target met on Harmonic: ~16× real time** at the default 5 ms step, steady
  across idle, driving, and rotating. RTF scales ~linearly with step size.
  5 ms is the default because it divides the 100 Hz controller period.
  Rotate-in-place gets ~7% faster at 5 ms than at 1 ms (servo response to a
  coarser step); forward speed doesn't change.
- **Mesh collision is Classic's problem, not Harmonic's.** DART keeps up with
  real time even with the full STL collisions. Primitives are the default
  anyway; they're what makes unthrottled stepping fast, and the terrain in
  Phase 2 will cost more contacts.
- **Classic's unthrottled RTF is state-dependent and unreliable**: 1.5–11× at
  1 ms depending on what the rover is doing (gz stats: 10.9 idle vs ~1.7 after
  driving). Two early bench readings (10.4, 40.9) came from that variance. Not
  pursued: Classic is only the Phase 0 reference now.
- **Rotate-in-place differs by simulator** (−84° upstream mesh, −103° Classic
  primitive, −116° Harmonic). Forward speed matches everywhere. Direction and
  sign match Phase 0. Contact geometry (mesh treads vs cylinders) and engine
  (ODE vs DART) both move it. Acceptable for RL; don't treat rotate rate as
  ground truth for the physical rover.
- **Bullet-Featherstone was tried and rejected**: 3.8× at 1 ms (vs DART 3.1) but
  the rover barely drives (0.23 m/s, −7° rotate). Joint control behaves
  differently there.
- **Harmonic bridges `/clock` on every physics step.** A Python node on sim time
  burned a full core just receiving it unthrottled. The rover node and command
  adapter never read the clock, so they run on wall time (as upstream runs them).
- **Humble `gazebo_ros2_control` can't take XML comments in `robot_description`**
  (it passes the URDF as an rcl command-line override). The launch file strips
  them.

Carried into Phase 2 (upstream model issues, left as-is for parity):

- **Wheel radius mismatch.** Mesh tyre radius is 0.082 m; `osr_params.yaml` says
  0.075. Every sim runs ~9% fast (0.3 → ~0.327). Decide which is right for the
  physical v4 rover before training on speed.
- **Upstream friction was never applied.** `<surface><friction>` inside URDF
  `<collision>` is dropped in URDF→SDF conversion (`gz sdf -p`: 6 `<mu>` in, 0
  out); wheels have always run on the simulator default. Set it deliberately with
  `<gazebo reference>` for Mars regolith.
- **The rocker pivot is fixed.** `rocker_bogie_joint_*_1` is `type="fixed"`; only
  the bogie articulates. Irrelevant on flat ground, wrong on Jezero terrain.
- **Primitive mode has no collision on rockers, bogies, or brackets.** Fine on
  flat ground; on rocks they need capsules/boxes so the rover can high-centre.

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
