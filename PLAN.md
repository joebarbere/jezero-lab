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

### Phase 2 results — 2026-09-29

```
ros2 launch osr_gz sim.launch.py world:=jezero_delta [realtime:=false]
```

**Terrain.** 256 m × 256 m of the Jezero delta front, 65.8 m of relief, from the
USGS Mars 2020 TRN HiRISE DTM mosaic (1 m/px, Fergason et al. 2020,
doi:10.5066/P9REJ9JN). `terrain/fetch_dtm.sh` reads just the window over HTTP
(the mosaic is uncompressed and row-striped, so a 257-row crop is ~22 MB of the
1.8 GB file, ~20 s). `terrain/build_jezero_delta.sh` rebuilds everything.

- **Where:** the 256 m window along Perseverance's traverse with the most climb
  while staying on the delta: sols 437–708, 41 route points. (The steepest window
  overall, sols 1244–1254 with 64 m of route climb, is the crater rim, left for later.)
- **Waypoints:** NASA's M20 waypoint file (MMGIS), lat/lon converted to the DTM's
  projection (x = R·lon, y = R·lat, R = 3,396,190 m). **NASA's recorded elevation
  matches the DTM within 0.1–0.4 m at every waypoint**, which validates the
  conversion. Five waypoints, sols 437 → 441 → 448 → 455 → 461: ~300 m of real
  route, climbing 16 m (z 3.3 → 19.3).
- **Slopes:** median 9.3°, 21% of cells > 15°, 7.5% > 20°, 0.9% > 30°, max 42.8°.
- **Loading:** 16-bit PNG heightmap with an explicit `<size>` (not a georeferenced
  DEM: Harmonic's DEM loader assumes Earth for geographic rasters, and the PNG
  path has fewer unknowns). Own regolith-coloured texture: Gazebo's stock terrain
  textures aren't installed with the Harmonic packages. Mars gravity 3.721 m/s².
  Waypoints are visual-only blue posts; spawn pose and waypoints are also in
  `worlds/jezero_delta.yaml` for Phase 3.

**Checks:**

| Check | Result |
|---|---|
| Heightmap orientation/scale | settled at z = 3.24 m at the sol 437 spawn; DTM says 3.29 m. Row-flipped would be 35.3, column-flipped 5.9 |
| Settling | pose identical over repeated reads after spawn on a ~7° slope: no sinking, bouncing, or sliding |
| Drive to sol 441 | straight at 0.3 m/s for 121 sim-s: 33.6 m, +3.6 m climb, ending 1.7 m from Perseverance's sol 441 position (33.2 m away). 0.28 m/s average; pitch up to 11° |
| RTF, unthrottled (gz `/stats`) | **Jezero 21–22×**, flat 15.5–16× (heightmap contact is cheaper for DART than the ground plane) |

**Model fixes (both simulators):**

- **Wheel radius:** the sim now passes `rover_dimensions.wheel_radius: 0.082` to
  the rover node (the simulated wheel; upstream's `rover_bringup.md` shows 0.082
  as the override). Forward speed went from 0.327 to **0.300 m/s** for 0.3
  commanded, on Harmonic and Classic.
- **Friction:** `wheel_mu:=` (default 0.7, upstream's intended μ) applied via
  `<gazebo reference>`, which does survive URDF→SDF (6 `<mu>` in the SDF, vs 0
  from upstream's `<surface>` block). Isotropic: upstream's μ2 = 0.3 has no
  defined direction without `fdir1`. No RTF cost (μ 0.7 vs 1.0: identical).
- Rotate-in-place is now −103° / 4 s real time on Harmonic, −94° on Classic (it
  goes through the wheel-radius math too).

**Tried and rejected: the rocker differential.** URDF `<mimic>` (left rocker =
−right) converts to SDF correctly, but DART in Harmonic refuses it at runtime:
"the chosen physics engine does not support mimic constraints". Without it the
body is free to pitch and flopped to −35°. Rockers stay fixed as upstream has them;
the bogies articulate, and the drive above shows that's enough on 1 m terrain.
Real fix: a small gz-sim system plugin applying the differential as a torque
coupling (τ ∝ −(θ_L + θ_R)). Bullet-Featherstone supports mimic but failed Phase 1's
drive test.

**Deferred to Phase 4 (boulder fields):** primitive collisions on rockers, bogies,
and brackets, so the rover can high-centre on rocks. A 1 m DTM has no rocks, so
nothing can touch them here.

**Tooling:** `tools/drive_test.py` (drive straight, log pose/tilt);
`tools/bench.py` now takes RTF from gz `/stats` on Harmonic, waits for `/cmd_vel`
to have a reader before timing (DDS discovery is ~1–2 s of wall time = tens of
sim-seconds unthrottled), and shuts its executor down cleanly.

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

### Phase 3 results — 2026-09-29

`jezero_env/`: a Gymnasium env with **no ROS in the training loop**. Run anything
in it through `jezero_env/run.sh` inside the `jazzy-harmonic` image.

**Spike outcome: in-process gz-sim, not a ROS bridge.** ROS couldn't give
deterministic stepping: /cmd_vel → rover node → adapter → controllers is async on
wall time, which at 15–20× RTF is many sim-steps of lag. Instead `jezero_env/sim.py`
runs the gz-sim server inside the Python process (`gz.sim8.TestFixture`), applies
commands to the joints in a pre-update callback every physics step, and
`step(n)` returns after exactly n steps. It uses the rover's own
`osr_control.kinematics` (pure Python, no ROS imports) and upstream's adapter sign
conventions, so an action drives the wheels exactly as `/cmd_vel` does.

Parity with the ROS stack, same 5 ms step: forward **0.300 m/s** (ROS 0.298–0.301),
rotate-in-place **−111°** / 4 s (ROS −112°), **14–15×** RTF in one process.

What it took (each of these silently broke the sim until found):

- **The gz Python bindings aren't in ROS's gz vendor packages.** They come from the
  OSRF apt repo (`python3-gz-sim8`, `python3-gz-transport13`, `libgz-sim8-plugins`),
  which installs a second, system copy of Harmonic (8.15) under `/usr`, next to
  ROS's vendor copy (8.11). The ROS image's `LD_LIBRARY_PATH` points at the vendor
  copy (undefined symbols), so `run.sh` sets `LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu`.
  The ROS stack is unaffected: it still resolves the vendor `gz` (bench after the
  change: 0.298 m/s, −112°), and the parity numbers above show the two minor
  versions behave the same. The dartsim engine
  plugin also needs its unversioned alias (`libgz-physics-dartsim-plugin.so`),
  which only the -dev package ships; the image links it.
- `import gz.math7` is required before `gz.sim8` returns poses, or pybind can't
  convert them (hard crash in the callback).
- **Per-instance `GZ_PARTITION`.** gz-transport is host-wide: two sims (in one
  process, or across a `SubprocVecEnv`) advertise the same `/world/<name>/…`
  services and answer each other's requests. Each `JezeroSim` sets its own.

**Reset — the 2020 bug — took three tries:**

1. `Server.reset_all()`: in gz-sim 8.15 it stops TestFixture's pre-update callback
   from firing afterwards. The rover resets, then ignores every command.
2. Teleport + zero every joint: `Link` velocity commands **persist every step**
   (the body hovered, pinned at zero velocity); without them, the body keeps its
   momentum through the teleport — after different histories, the same episode
   drifted apart by 8 cm in 2 s.
3. **Delete the rover and spawn a fresh one** (`/world/<name>/remove` + `create`,
   called while the server steps in the background — the bindings release the
   GIL, so it doesn't deadlock). A service reply only means "queued", so reset
   steps until the old model is gone and the new one exists before continuing.

Result: after three different histories (driving, spinning, reversing while
turning), the following episode is **identical: 0.000 mm, 0.0000° spread**. Reset
takes 160–175 ms. Gymnasium's `check_env` passes, including its same-seed reset
check.

**Settling after spawn.** The first soak (1,000 random episodes) scored 29% of
episodes as tipped. All of them tipped on **step 1**, at 40–46° pitch, and 26 of
27 were back to ~10° (the terrain slope) 3 s later: the rover was still rocking
from the drop when the episode started. With fixed rockers the body pivots on the
undamped bogie joints, and at 0.38 g that swing is slow; the fixed 1 s settle
wasn't enough. Reset now settles until the body is still (speed < 0.01 m/s,
angular speed < 0.02 rad/s for 0.5 s; median 3.0 sim-s, p95 4.2), and the env
respawns with another heading if a spawn never settles (a heading across a steep
spot). Tips from random driving after that: **0 in 120 episodes** of the
characterisation run.

**The env** (`jezero_env/env.py`):

- Action `[speed, turn]` ∈ [−1, 1]² → 0.3 m/s, 0.6 rad/s → kinematics. 5 Hz control
  (40 physics steps of 5 ms).
- Observation, 61 floats: goal in rover frame + distance, heading error (sin/cos),
  roll, pitch, body-frame velocities and yaw rate, previous action, and a 7×7 terrain
  patch at 1 m spacing relative to the rover's height, sampled from the same
  heightmap the physics collides with (rover z 3.290 vs heightmap 3.300 at spawn).
- Reward, from 2020: progress toward the goal, a goal bonus scaled by budget left,
  `at_destination()` bounding box (±1 m), per-step time cost; terminate with a
  penalty on tipping past 35° or leaving the map; truncate when the step budget
  (the "power supply": 2.5× straight-line time) runs out.
- Reset settles the rover until it is still, and respawns with another heading if
  it never does (see above). `info` reports `settle_time` and `spawn_attempts`.
- Episodes are segments of Perseverance's route between waypoints; `segments=` and
  `random_heading=` pick the start.

**Baseline** (`python3 -m jezero_env.baseline`): turn toward the goal, drive, slow
when misaligned. **Reaches every goal, all 4 segments, the whole 235 m route**,
within ~1 m, using ~40% of the budget:

| Segment | Distance | Result | Sim time | Wall |
|---|---|---|---|---|
| sol 437 → 441 | 33.3 m | goal, 540/1385 steps | 108 s | 5.5 s |
| sol 441 → 448 | 104.7 m | goal, 1731/4362 | 346 s | 18.0 s |
| sol 448 → 455 | 53.8 m | goal, 882/2238 | 176 s | 9.1 s |
| sol 455 → 461 | 43.3 m | goal, 716/1802 | 143 s | 7.4 s |

**The task is too easy as-is**: on smooth 1 m terrain a trivial controller succeeds
100% from the default spawns, so "beat the baseline on success rate" can't be
Phase 4's bar here. See Phase 4.

**Soak — done-criterion met** (`python3 -m jezero_env.soak 1000 20`): 1,000
random-agent episodes (random segment and heading, 20 sim-s cap), 100,000 steps in
1,234 s:

| | |
|---|---|
| Hangs / crashes | none |
| Memory | +33 MB warm-up in the first 100 episodes, then +2 MB over the next 900 (~2 KB/episode: allocator noise, not a leak); peak 237 MB |
| Throughput | 81–84 env steps/s (5 Hz control → ~16× real time), one process |
| Reset | median ~0.2 s wall; worst 1.3 s (a respawn) |
| Respawns | 39 (3.9%) needed a second heading to settle |
| Terminations | 0 tipped, 0 out of bounds; all 1,000 hit the 20 s cap |

## Phase 4 — Training

- Parallel envs as separate processes (`SubprocVecEnv`); each `JezeroSim` already
  takes its own `GZ_PARTITION`, and there's no ROS in the loop, so no
  `ROS_DOMAIN_ID` needed. Start with 6–8, then tune based on CPU use.
- PPO first, then SAC. TensorBoard (replaces the 2020 ELK stack).
- Curriculum: flat ground, then the delta slope, then boulder fields.
- **Harder variants, because the baseline already succeeds 100% on the plain
  route:** random start headings, spawn points off the route, boulder fields (and
  primitive collisions on rockers/bogies so high-centring is possible), steeper
  terrain (the crater rim, sols 1244–1254).
- **Done when:** the policy beats the hand-written go-to-goal controller
  (`jezero_env/baseline.py`) on success rate on the harder variants, across
  held-out spawn points, and on time-to-goal on the plain route.
- Throughput to plan around: ~82 env steps/s per process. At 6–8 processes, ~500–650
  env steps/s, i.e. ~2M steps/hour, before PPO's own overhead.
- Worth trying: a little damping on the bogie joints. Real pivots have friction,
  and it would shorten the 3 s post-spawn settle that dominates reset time.

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
