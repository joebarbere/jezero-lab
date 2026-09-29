#!/usr/bin/env bash
# Run a phase image with the host display and GPU.
#   containers/run.sh [flavor] [command...]
# Default command is an interactive shell. Open a second shell into the same
# container with: podman exec -it jezero-lab bash
set -euo pipefail

flavor="${1:-humble-classic}"
shift || true
[ $# -eq 0 ] && set -- bash

# Gazebo Classic and RViz are X11 apps; on Wayland they go through XWayland.
: "${DISPLAY:?DISPLAY not set — run from a graphical session}"

# GNOME's XWayland requires a cookie. Mount it rather than opening the display
# with `xhost +`.
xauth_args=()
if [ -n "${XAUTHORITY:-}" ] && [ -f "$XAUTHORITY" ]; then
    xauth_args=(-e XAUTHORITY=/tmp/.Xauthority -v "$XAUTHORITY:/tmp/.Xauthority:ro")
fi

tty_args=(-i)
[ -t 0 ] && tty_args=(-it)

exec podman run --rm "${tty_args[@]}" --name jezero-lab \
    --device /dev/dri \
    --group-add keep-groups \
    --security-opt label=disable \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    "${xauth_args[@]}" \
    --ipc host \
    "jezero-lab:${flavor}" "$@"
