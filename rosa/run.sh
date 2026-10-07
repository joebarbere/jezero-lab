#!/usr/bin/env bash
# Talk to the simulated OSR in plain language: starts Gazebo (jezero_delta by
# default) and the ROSA agent in one container.
#   ANTHROPIC_API_KEY=... rosa/run.sh [--gui] [world]
# Build the image once: podman build -f rosa/Containerfile -t jezero-lab:rosa .
set -euo pipefail
cd "$(dirname "$0")/.."
[ -n "${ANTHROPIC_API_KEY:-}" ] || { echo "Set ANTHROPIC_API_KEY" >&2; exit 1; }
gui=false; opts=()
if [ "${1:-}" = --gui ]; then
    shift; gui=true
    opts=(-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --ipc host --device /dev/dri
          --group-add keep-groups --security-opt label=disable)
    [ -n "${XAUTHORITY:-}" ] && opts+=(-e XAUTHORITY=/tmp/.Xauthority -v "$XAUTHORITY:/tmp/.Xauthority:ro")
fi
world="${1:-jezero_delta}"
exec podman run --rm -it "${opts[@]}" -e ANTHROPIC_API_KEY -e ROSA_MODEL -e JEZERO_WORLD="$world" \
    -v "$PWD:/repo:z" -w /repo jezero-lab:rosa bash -c "
source /opt/ros/jazzy/setup.bash && source /osr_ws/install/setup.bash
ros2 launch osr_gz sim.launch.py world:=$world gui:=$gui > /tmp/sim.log 2>&1 &
echo 'Starting the simulator...'
for i in \$(seq 90); do timeout 8 ros2 control list_controllers 2>/dev/null | grep -q active && break; sleep 2; done
python3 rosa/osr_agent.py"
