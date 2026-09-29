#!/usr/bin/env bash
# Launch upstream's osr_gazebo headless from a local checkout and run
# tools/bench.py against it.  upstream/bench_upstream.sh <flavor> [launch args...]
set -euo pipefail
cd "$(dirname "$0")/.."
flavor="$1"; shift
upstream/osr_run.sh "$flavor" "
(ros2 launch osr_gazebo empty_world.launch.py gui:=false $* > /tmp/launch.log 2>&1 &)
for i in \$(seq 40); do
  n=\$(timeout 8 ros2 control list_controllers 2>/dev/null | grep -c active || true)
  [ \"\$n\" -ge 3 ] && break; sleep 3
done
if [ \"\${n:-0}\" -lt 3 ]; then echo 'controllers not active:'; grep -iE 'error|fail' /tmp/launch.log | head -20; exit 1; fi
timeout 120 python3 /tools/bench.py
"
