# Contributions to nasa-jpl/osr-rover-code

Work prepared from jezero-lab for the upstream OSR repo. Code lives on branches of
a separate checkout, `../osr-rover-code` (not the pinned submodule); this folder
holds the test harness and the issue/PR drafts. Nothing here has been sent.

| Draft | Branch | Status |
|---|---|---|
| [issue_harmonic_plan.md](issue_harmonic_plan.md) | – | ask first: keep Classic alongside Harmonic, or replace it? |
| [pr2_gazebo_harmonic.md](pr2_gazebo_harmonic.md) | `feature/gazebo-harmonic` | ready after the issue is answered |
| [pr1_wheel_friction.md](pr1_wheel_friction.md) | `fix/gazebo-wheel-friction` | ready |
| [issue_wheel_radius.md](issue_wheel_radius.md) | – | ready (a question, not a PR) |

Harness (all build the checkout inside a jezero-lab image):

- `osr_run.sh <humble-classic|jazzy-harmonic> [--gui] <cmd>`: build and run a command
- `bench_upstream.sh <flavor>`: upstream's `empty_world.launch.py`, headless, then `tools/bench.py`
- `slope_test.sh <flavor>`: rover on a 10° slope, wheels held, how far it slides (friction test)
