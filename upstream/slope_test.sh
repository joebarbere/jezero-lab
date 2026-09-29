#!/usr/bin/env bash
# Does wheel friction reach the simulator? Spawn the rover on a 10-degree slope
# with the wheels held at zero velocity and see whether it slides.
#   upstream/slope_test.sh <flavor>
# Classic combines the two surfaces' mu with min(), so the slope is given mu=100
# and the wheels' value decides.
set -euo pipefail
cd "$(dirname "$0")/.."
upstream/osr_run.sh "$1" "
(ros2 launch osr_gazebo empty_world.launch.py gui:=false world:=/tools/slope_10deg.world > /tmp/launch.log 2>&1 &)
for i in \$(seq 40); do
  n=\$(timeout 8 ros2 control list_controllers 2>/dev/null | grep -c active || true)
  [ \"\$n\" -ge 3 ] && break; sleep 3
done
p0=\$(gz model -m rover -p | head -1); t0=\$(gz stats -p -d 1 2>/dev/null | tail -1 | cut -d, -f2)
sleep 40
p1=\$(gz model -m rover -p | head -1); t1=\$(gz stats -p -d 1 2>/dev/null | tail -1 | cut -d, -f2)
python3 -c \"
a=[float(v) for v in '\$p0'.split()]; b=[float(v) for v in '\$p1'.split()]
dt=float('\$t1')-float('\$t0')
print(f'slid {b[0]-a[0]:+.3f} m downhill (x) in {dt:.1f} sim-s')\"
"
