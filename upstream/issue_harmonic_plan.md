# Issue draft: Plan for moving `osr_gazebo` to Gazebo Harmonic / `ros_gz` (Jazzy)

Target: `nasa-jpl/osr-rover-code` issues. Asks the maintainers how they'd like the migration done before sending the PR.

---

`osr_gazebo/README.md` lists "Migrate from Gazebo Classic to Harmonic / `ros_gz` for Jazzy" as a follow-up, and #223 moves the docs to Jazzy LTS. Gazebo Classic reached end of life in January 2025 and isn't available on Jazzy. I have a working port and would like to contribute it, but first a question about the shape you'd prefer.

**What the port does** (tested on Humble + Classic 11 and Jazzy + Harmonic 8):
- `gazebo.urdf.xacro` takes a `sim` argument (`classic` | `harmonic`) and emits either `gazebo_ros2_control/GazeboSystem` + the `gazebo_ros` IMU plugin, or `gz_ros2_control/GazeboSimSystem` + the `gz-sim` IMU system.
- `empty_world.launch.py` picks the simulator from `ROS_DISTRO` (Humble → Classic, Jazzy → Harmonic). The Harmonic branch uses `ros_gz_sim` to start Gazebo and spawn the robot, and `ros_gz_bridge` for `/clock` and the IMU.
- Unchanged: the model, the controllers and their config, `gazebo_command_adapter.py`, and the `rover` node + `kinematics.py` path from #224.
- `package.xml` dependencies are conditional on `ROS_DISTRO`, so rosdep resolves the right set on each.

Measured, same commands on both (0.3 m/s forward, then `angular.y` 0.5 in place):

| | Humble + Classic | Jazzy + Harmonic |
|---|---|---|
| Controllers active | yes | yes |
| Real-time factor | 0.26 (unchanged from `master`) | 0.99 |
| Forward speed | 0.322 m/s (`master`: 0.321) | 0.327 m/s |
| Rotate in place, 4 s | −83.2° (`master`: −84.1°) | −118° |

A clean `ros:jazzy-ros-base` container with only the README's apt line builds, launches and drives it.

**The question:** keep Classic alongside Harmonic (the approach above: Humble users keep working, Jazzy users get Harmonic), or replace Classic outright and target Jazzy only, to match #223? Keeping both costs a `sim` switch in two files; replacing it is simpler to maintain.

Separately, and only if you're interested: the model uses the full-resolution STL meshes as collision geometry (about 1.4M triangles), which holds Gazebo Classic to ~0.25× real time. Simple primitives for collision (cylinders for the wheels, a box for the body; meshes kept for visuals) run it at real time. Harmonic's physics copes with the meshes, so this matters mainly for Classic. I can send it as a separate PR behind an option.
