# Contributions to nasa-jpl/osr-rover-code

Work prepared from jezero-lab for the upstream OSR repo. Code lives on branches of
a separate checkout, `../osr-rover-code` (not the pinned submodule), pushed to the
fork [joebarbere/osr-rover-code](https://github.com/joebarbere/osr-rover-code);
this folder holds the test harness and the issue/PR drafts.

| Draft | Branch | Status (2026-09-29) |
|---|---|---|
| [issue_harmonic_plan.md](issue_harmonic_plan.md) | – | **opened: [#228](https://github.com/nasa-jpl/osr-rover-code/issues/228)**, asks: keep Classic alongside Harmonic, or replace it? |
| [pr2_gazebo_harmonic.md](pr2_gazebo_harmonic.md) | `feature/gazebo-harmonic` (pushed) | held until #228 is answered |
| [pr1_wheel_friction.md](pr1_wheel_friction.md) | `fix/gazebo-wheel-friction` (pushed) | **opened: [#230](https://github.com/nasa-jpl/osr-rover-code/pull/230)** |
| [issue_wheel_radius.md](issue_wheel_radius.md) | – | **opened: [#229](https://github.com/nasa-jpl/osr-rover-code/issues/229)** |

Commits carry a `Co-Authored-By: Claude` trailer and the bodies end with a
"Generated with Claude Code" line; the private session link is left out.

Harness (all build the checkout inside a jezero-lab image):

- `osr_run.sh <humble-classic|jazzy-harmonic> [--gui] <cmd>`: build and run a command
- `bench_upstream.sh <flavor>`: upstream's `empty_world.launch.py`, headless, then `tools/bench.py`
- `slope_test.sh <flavor>`: rover on a 10° slope, wheels held, how far it slides (friction test)
