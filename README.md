# jezero-lab

Local reinforcement-learning lab for the NASA-JPL Open Source Rover v4, driving a
simulated patch of Jezero Crater. Runs entirely on one machine in Podman: no cloud,
no NVIDIA GPU required.

See [PLAN.md](PLAN.md) for the phases. **Current phase: 0** (upstream `osr_gazebo`
on ROS 2 Humble + Gazebo Classic, as a known-good baseline).

## Setup

```bash
git clone --recurse-submodules <this repo>
containers/build.sh humble-classic
```

`ros_ws/src/osr-rover-code` is a submodule of
[nasa-jpl/osr-rover-code](https://github.com/nasa-jpl/osr-rover-code), pinned to a
specific commit. The image copies its `ROS/` packages into `/osr_ws` and removes
upstream's `COLCON_IGNORE` from `osr_gazebo` inside the image; the submodule itself
stays untouched.

## Phase 0: drive the rover

```bash
containers/run.sh humble-classic
# inside the container:
ros2 launch osr_gazebo empty_world.launch.py
```

In a second terminal:

```bash
podman exec -it jezero-lab bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Rotate-in-place uses `Twist.angular.y`, which `teleop_twist_keyboard` can't send.
Test it with:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {y: 0.5}}"
```

For no Gazebo window, add `gui:=false` (it passes through to `gazebo.launch.py`).

`run.sh` passes the host `DISPLAY`, the X11 socket, the XWayland auth cookie, and
`/dev/dri`, so Gazebo renders on the host GPU through Mesa (verified on an AMD
RX 7600: `GFX1102`, hardware OpenGL 4.6).

Known issue: the upstream sim runs at **0.25× real time** because its collision
geometry is full-resolution meshes. See PLAN.md, Phase 0 results.
