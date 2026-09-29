#!/usr/bin/env bash
# Run a Python entry point with the environment jezero_env needs, inside the
# jazzy-harmonic image:
#   jezero_env/run.sh python3 -m jezero_env.selftest
#
# - The system gz-sim (/usr, OSRF packages) that the gz.sim8 bindings are built
#   against. The ROS image puts ROS's vendor copy first in LD_LIBRARY_PATH;
#   loading that instead fails with undefined symbols.
# - osr_control (pure-Python kinematics) from the colcon install, without
#   sourcing ROS.
set -euo pipefail
cd "$(dirname "$0")/.."
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu
export PYTHONPATH="$PWD:/osr_ws/install/osr_control/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
unset GZ_CONFIG_PATH GZ_SIM_SYSTEM_PLUGIN_PATH GZ_SIM_PHYSICS_ENGINE_PATH
# Rendering (the chase camera): the OSRF ogre2 package doesn't ship its shader and
# media files; ROS's vendor copy has them for the same gz-rendering version (8.2.3).
export GZ_RENDERING_RESOURCE_PATH=/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz/gz-rendering8
exec "$@"
