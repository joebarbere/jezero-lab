# Talk to the rover: ROSA + the simulated OSR

[ROSA](https://github.com/nasa-jpl/rosa) is NASA-JPL's LLM agent for ROS: you ask
questions or give commands in plain language and it uses ROS (topics, nodes,
services, parameters, logs) to answer or act. This connects it, with Claude, to
the simulated Open Source Rover on Jezero terrain.

```bash
podman build -f rosa/Containerfile -t jezero-lab:rosa .      # once
ANTHROPIC_API_KEY=... rosa/run.sh --gui                       # or headless, or: rosa/run.sh jezero_rim
```

Things to try:

- "Where are you, and what's the nearest waypoint?"
- "List Perseverance's waypoints and drive to the closest one."
- "Turn to face sol 448, then drive 10 m and tell me how steep it got."
- "Which ROS nodes and controllers are running? Is anything publishing on /cmd_vel?"

Rover tools ([osr_agent.py](osr_agent.py)): `rover_status`, `list_waypoints`,
`drive_distance` (≤ 30 m), `rotate_in_place`, `drive_to` / `drive_to_waypoint`
(simple go-to-goal steering; it does not plan around rocks), `stop`. Every drive
stops on its own past 25° of tilt or on timeout, and the agent is told to report
those stops rather than retry. ROSA's own tools cover ROS inspection.

Settings: `ROSA_MODEL` (default `claude-sonnet-5`), world as the last argument
(`jezero_delta` default; any world in `ros_ws/src/osr_gz/worlds`).

This is the ROS side of the project (the same launch as teleop); RL training
steps Gazebo in-process and doesn't use ROS, so ROSA isn't involved there.

Tested: the rover tools against the headless simulator (drive 3 m, rotate
±45/90/180° to within 4°, drive to sol 441 33 m away, arriving within 1.3 m).
