# Issue draft: Gazebo wheel is 0.082 m but `osr_params.yaml` says 0.075 m, so the sim drives ~9% fast

Target: `nasa-jpl/osr-rover-code` issues.

---

Since #224 the simulation drives through the same `rover` node and `kinematics.py` as the hardware, with dimensions from `osr_bringup/config/osr_params.yaml`. That file has `wheel_radius: 0.075`, but the wheel in the Gazebo model (`osr_gazebo/meshes/wheel.stl`) has a radius of **0.082 m** (bounding box 0.164 × 0.101 × 0.164 m).

The rover node converts a commanded speed into wheel angular velocity using 0.075, and the simulated wheel then covers ground at 0.082, so the sim runs 0.082 / 0.075 = 1.09× faster than commanded:

| `/cmd_vel` linear.x | Measured in Gazebo (Classic, Humble) |
|---|---|
| 0.3 m/s | 0.321–0.325 m/s |

Passing `rover_dimensions.wheel_radius: 0.082` to the `rover` node in the Gazebo launch brings it to 0.300 m/s.

`setup/rover_bringup.md` already uses 0.082 as its example of overriding the wheel radius, so I'm not sure which value is right for the current v4 hardware. Two ways to line them up:

1. If the v4 wheel really is 0.082 m, change `osr_params.yaml` (this also changes speeds on hardware).
2. If 0.075 m is right for the hardware, keep it and override `wheel_radius` in the Gazebo launch file to match the simulated wheel, or scale the mesh.

Happy to open a PR for whichever you prefer.
