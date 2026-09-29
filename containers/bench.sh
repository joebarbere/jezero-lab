#!/usr/bin/env bash
# Launch a sim headless in a fresh container, wait for it, run tools/bench.py.
#   containers/bench.sh <flavor> <launch args...>
# e.g.
#   containers/bench.sh humble-classic osr_gazebo empty_world.launch.py gui:=false
#   containers/bench.sh humble-classic osr_gz sim.launch.py gui:=false collision:=mesh
#   containers/bench.sh jazzy-harmonic osr_gz sim.launch.py gui:=false
set -euo pipefail
cd "$(dirname "$0")/.."

flavor="$1"; shift
name="jezero-bench-$$"

podman run -d --rm --name "$name" \
    --device /dev/dri --security-opt label=disable \
    -v "$PWD/tools:/tools:ro" \
    "jezero-lab:${flavor}" \
    bash -lc "source /osr_ws/install/setup.bash && ros2 launch $*" >/dev/null
trap 'podman rm -f "$name" >/dev/null 2>&1 || true' EXIT

# Wait until all three controllers are active (or give up after 120 s).
# Each call gets its own timeout: list_controllers blocks forever when there
# is no controller manager.
for _ in $(seq 8); do
    active=$(timeout 15 podman exec "$name" bash -lc \
        'source /osr_ws/install/setup.bash && timeout 8 ros2 control list_controllers 2>/dev/null' \
        | grep -c active || true)
    [ "$active" -ge 3 ] && break
    sleep 2
done
if [ "$active" -lt 3 ]; then
    echo "controllers not active after 120 s; last log lines:" >&2
    podman logs --tail 40 "$name" >&2
    exit 1
fi

echo "== ${flavor}: $*"
podman exec "$name" bash -lc 'source /osr_ws/install/setup.bash && python3 /tools/bench.py'
