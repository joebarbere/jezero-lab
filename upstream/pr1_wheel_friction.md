# PR draft: Apply Gazebo wheel friction through `<gazebo reference>`

Branch: `fix/gazebo-wheel-friction` (1 commit on `6b17c22`). Target: `nasa-jpl/osr-rover-code` `master`.

---

## Summary
- Move the wheels' friction from `<surface><friction>` inside the URDF `<collision>` elements to `<gazebo reference>` `mu1`/`mu2` on the six wheel links, set from one pair of xacro properties.
- Use 0.7 for both directions.

## Motivation
URDF has no `<surface>` element, so the URDF → SDF conversion drops it and the friction values never reached Gazebo. Converting the xacro output shows it:

```
xacro osr.urdf.xacro > r.urdf && gz sdf -p r.urdf | grep -c '<mu>'
# master: 0        this branch: 6
```

The wheels have been running on Gazebo's default friction (μ = 1) whatever the URDF said.

On the values: the old `mu2 = 0.3` only means something along a friction direction (`fdir1`), which was never set, so which way the lower friction applied was up to the physics engine. This branch uses 0.7 in both directions. If an anisotropic value is wanted (e.g. lower lateral friction), it needs `fdir1` set along the wheel axle as well; happy to add that instead.

## Test plan
Gazebo Classic 11 / Humble, headless, driving through the `rover` node as in the README.

**Friction reaches the simulator.** The rover spawned on a 10° slope with the wheels held at zero velocity, then left for 2.2 s of sim time. A friction of 0.05 is below tan 10° ≈ 0.18, so if the value reaches Gazebo the rover should slide:

| Build | Wheel friction | Slid downhill |
|---|---|---|
| `master` | as shipped (0.7 / 0.3) | 0.18 m |
| `master` | 0.05 in the `<surface>` block | 0.18 m (no effect) |
| this branch | 0.05 | **4.13 m** |
| this branch | 0.7 (default) | 0.19 m |

(The ~0.18 m in every case is the wheel velocity controller's creep, not friction.)

**Driving is unchanged on flat ground** (`teleop`-equivalent: `/cmd_vel` 0.3 m/s forward, then `angular.y` 0.5 in place):

| Build | Forward speed | Rotate in place (4 s) |
|---|---|---|
| `master` | 0.321 m/s | −84.1° |
| this branch | 0.321 m/s | −85.0° |

- [x] `colcon build --packages-select osr_interfaces osr_control osr_bringup osr_gazebo` on Humble (with `osr_gazebo/COLCON_IGNORE` removed)
- [x] `ros2 launch osr_gazebo empty_world.launch.py`, drive forward and rotate in place
- [x] Slope test above
- [ ] `ros2 launch osr_gazebo rviz.launch.py` (unaffected: visual/collision geometry unchanged)
