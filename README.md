# jezero-lab

Local reinforcement-learning lab for the NASA-JPL Open Source Rover v4, driving a
simulated patch of Jezero Crater. Runs entirely on one machine in Podman: no cloud,
no NVIDIA GPU required.

See [PLAN.md](PLAN.md) for the phases and measured results. **Current phase: 3**
(Gym environment). Done so far: the OSR runs on ROS 2 Jazzy + Gazebo Harmonic, on
256 m of real Jezero delta terrain from HiRISE, with waypoints on Perseverance's
actual route, at ~20× real time headless.

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

## Terrain

`terrain/build_jezero_delta.sh` rebuilds the `jezero_delta` world from source: a
257 m window of the USGS Mars 2020 HiRISE DTM mosaic (1 m/px, ~22 MB read over
HTTP, not the 1.8 GB file) and NASA's Perseverance waypoint file. The generated
heightmap, world, and waypoint yaml are committed, so this is only needed to change
the area.

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
- `terrain/`: DTM crop, heightmap/world generator, rebuild script.
- `tools/`: benchmark and drive test.

`run.sh` passes the host `DISPLAY`, the X11 socket, the XWayland auth cookie, and
`/dev/dri`, so Gazebo renders on the host GPU through Mesa (verified on an AMD
RX 7600: `GFX1102`, hardware OpenGL 4.6).

## License

Apache-2.0, matching upstream. `osr_gz` is derived from `osr-rover-code`
(Copyright 2018 California Institute of Technology); see [NOTICE](NOTICE).
