#!/usr/bin/env bash
# Build a local osr-rover-code checkout inside a jezero-lab image and run a
# command against it. For preparing upstream contributions: tests the clone as
# upstream would use it (their packages, their launch files), not osr_gz.
#
#   upstream/osr_run.sh <humble-classic|jazzy-harmonic> [--gui] <command...>
#
# OSR_SRC defaults to ../osr-rover-code next to this repo. The checkout is
# mounted read-only and copied into /ws/src; osr_gazebo's COLCON_IGNORE is
# removed in the copy, as its README says to.
set -euo pipefail
cd "$(dirname "$0")/.."
flavor="$1"; shift
gui=()
if [ "${1:-}" = --gui ]; then
    shift
    gui=(-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --ipc host)
    [ -n "${XAUTHORITY:-}" ] && gui+=(-e XAUTHORITY=/tmp/.Xauthority -v "$XAUTHORITY:/tmp/.Xauthority:ro")
fi
src="${OSR_SRC:-$(cd .. && pwd)/osr-rover-code}"
distro=humble; [ "$flavor" = jazzy-harmonic ] && distro=jazzy
exec podman run --rm -i "${gui[@]}" --device /dev/dri --group-add keep-groups \
    --security-opt label=disable --name "osr-${flavor}-$$" \
    -v "$src:/osr_src:ro" -v "$PWD/tools:/tools:ro" "jezero-lab:${flavor}" bash -lc "
set -e
mkdir -p /ws/src && cp -r /osr_src/ROS/. /ws/src/ && rm -f /ws/src/osr_gazebo/COLCON_IGNORE
source /opt/ros/${distro}/setup.bash
cd /ws && colcon build --packages-select osr_interfaces osr_control osr_bringup osr_gazebo \
    --event-handlers console_direct- > /tmp/build.log 2>&1 || { tail -40 /tmp/build.log; exit 1; }
source /ws/install/setup.bash
$*"
