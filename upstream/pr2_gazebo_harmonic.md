# PR draft: Support Gazebo Harmonic on Jazzy in `osr_gazebo`

Branch: `feature/gazebo-harmonic` (1 commit on `6b17c22`). Target: `nasa-jpl/osr-rover-code` `master`.
Send after the maintainers answer #228 (keep Classic alongside, or replace it); reference it in the PR.
Independent of `fix/gazebo-wheel-friction`; the two merge cleanly in either order.

---

## Summary
- `osr_gazebo` now runs on **ROS 2 Jazzy with Gazebo Harmonic**, and still on Humble with Gazebo Classic 11. `empty_world.launch.py` picks the simulator from `ROS_DISTRO`.
- `osr.urdf.xacro` takes a `sim` argument (`classic` by default); `gazebo.urdf.xacro` emits `gazebo_ros2_control` + the `gazebo_ros` IMU plugin for Classic, or `gz_ros2_control` + the `gz-sim` IMU system for Harmonic. The Harmonic IMU publishes on the same topic, `imu_plugin/out`.
- On Harmonic the launch file starts Gazebo through `ros_gz_sim`, spawns with `ros_gz_sim create`, and bridges `/clock` and the IMU with `ros_gz_bridge`. A `gui` argument works on both.
- `package.xml` (format 3) makes the simulator dependencies conditional on `ROS_DISTRO`, so rosdep installs the right set, and adds `ros2controlcli`: the launch file's `ros2 control load_controller` calls need it, and Jazzy doesn't install it by default.
- README: dependencies and install steps for both distros; removes the migration follow-up.

Unchanged: the model, the controllers and `controller_velocity.yaml`, `gazebo_command_adapter.py`, and the `rover` node + `kinematics.py` path from #224.

## Motivation
Gazebo Classic reached end of life in January 2025 and isn't packaged for Jazzy, which the hardware docs already target (#223). The migration was listed as a follow-up in `osr_gazebo/README.md`.

## Architecture
Same as #224 on both simulators:
`/cmd_vel` → `rover` (`kinematics` + `osr_params.yaml`) → `/cmd_drive` + `/cmd_corner` → `gazebo_command_adapter` → `wheel_controller` / `servo_controller`, now via `gz_ros2_control` on Harmonic.

## Test plan
Headless, `ros2 launch osr_gazebo empty_world.launch.py gui:=false`, then `/cmd_vel` 0.3 m/s forward, then `angular.y` 0.5 (rotate in place), measured against sim time:

| | `master`, Humble + Classic | this branch, Humble + Classic | this branch, Jazzy + Harmonic |
|---|---|---|---|
| Controllers active (3) | ✓ | ✓ | ✓ |
| Real-time factor | 0.26 | 0.26 | 0.99 |
| Forward speed | 0.321 m/s | 0.322 m/s | 0.327 m/s |
| Rotate in place, 4 s | −84.1° | −83.2° | −118° |
| IMU on `/imu_plugin/out` | ✓ | ✓ | ✓ |

Rotate-in-place is faster on Harmonic: different physics engine (DART vs ODE) and wheel contact with the same mesh. Direction and forward speed agree. (Forward speed ~9% above commanded on both: the wheel-radius mismatch in #229.) The low Classic real-time factor is pre-existing, from the full-resolution collision meshes; Harmonic's physics isn't slowed by them.

- [x] `colcon build --packages-select osr_interfaces osr_control osr_bringup osr_gazebo` on Humble and on Jazzy
- [x] **Clean Jazzy install from the README:** `ros:jazzy-ros-base` container, only the README's `apt install` line, build, launch, drive: controllers active, same numbers as above
- [x] `rosdep keys` / `rosdep check`: Classic keys on Humble, Harmonic keys on Jazzy
- [x] `ros2 launch osr_gazebo rviz.launch.py` on both (joint_state_publisher_gui, robot_state_publisher, rviz2 start)
- [x] `colcon test --packages-select osr_gazebo`: no new failures on either distro (`master` already has 62 on Humble / 64 on Jazzy, all in files this PR doesn't touch)
- [ ] Drive with `teleop_twist_keyboard` in the Harmonic GUI (tested headless here)
