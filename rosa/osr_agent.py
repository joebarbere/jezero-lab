#!/usr/bin/env python3
"""Talk to the simulated Open Source Rover in plain language with ROSA
(NASA-JPL's LLM agent for ROS, https://github.com/nasa-jpl/rosa) and Claude.

    rosa/run.sh            # starts the sim on jezero_delta and this agent

ROSA brings its own tools for inspecting ROS (topics, nodes, services,
parameters, logs). This adds rover tools: status, Perseverance's waypoints on
the current world, driving a distance, rotating in place, driving to a point or
waypoint, and stopping. Every drive stops on its own if the rover tilts past
TILT_LIMIT_DEG or takes too long.

Environment: ANTHROPIC_API_KEY (required), ROSA_MODEL (default claude-sonnet-5),
JEZERO_WORLD (default jezero_delta).
"""
import math
import os
import re
import subprocess
import sys
import threading
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from langchain.agents import tool

TILT_LIMIT_DEG = 25.0      # the RL environment counts > 35 deg as tipped over
SPEED = 0.3                # m/s, the rover's normal driving speed in the sim
ROTATE_RATE = 0.5          # rad/s for rotate-in-place (Twist.angular.y on the OSR)
ARRIVED_M = 1.5
NUM = r'[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?'
WORLD = os.environ.get('JEZERO_WORLD', 'jezero_delta')

_node = None
_pub = None


def _ros():
    """One rclpy node, spun in a background thread, publishing /cmd_vel."""
    global _node, _pub
    if _node is None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        rclpy.init()
        _node = rclpy.create_node('rosa_osr_tools')
        _pub = _node.create_publisher(Twist, '/cmd_vel', 10)
        ex = SingleThreadedExecutor()
        ex.add_node(_node)
        threading.Thread(target=ex.spin, daemon=True).start()
    return _pub


def _pose():
    """(x, y, z, roll, pitch, yaw) of model 'rover' from the gz CLI (gz-sim 8).
    The CLI occasionally returns nothing on a busy machine, so retry."""
    for _ in range(5):
        try:
            out = subprocess.run(['gz', 'model', '-m', 'rover', '-p'],
                                 capture_output=True, text=True, timeout=20).stdout
        except subprocess.TimeoutExpired:
            continue
        vecs = re.findall(r'\[\s*(' + NUM + r')\s+(' + NUM + r')\s+(' + NUM + r')\s*\]', out)
        if len(vecs) >= 2:
            return tuple(float(v) for v in (*vecs[0], *vecs[1]))
        time.sleep(0.2)
    _halt()
    raise RuntimeError('could not read the rover pose: is the simulator running?')


def _tilt_deg(p):
    return math.degrees(max(abs(p[3]), abs(p[4])))


def _waypoints():
    path = os.path.join(get_package_share_directory('osr_gz'), 'worlds', WORLD + '.yaml')
    if not os.path.exists(path):
        return []
    return (yaml.safe_load(open(path)) or {}).get('waypoints') or []


def _send(linear=0.0, steer=0.0, rotate=0.0):
    """steer, rotate: rad/s, positive = counter-clockwise (left). The OSR's
    rotate-in-place command (Twist.angular.y) turns clockwise for positive
    values, so it is negated here."""
    t = Twist()
    t.linear.x, t.angular.z, t.angular.y = float(linear), float(steer), -float(rotate)
    _ros().publish(t)


def _halt():
    if _pub is None:
        return
    for _ in range(5):
        _send()
        time.sleep(0.05)


def _status_line(p):
    return (f'x={p[0]:.1f} m, y={p[1]:.1f} m, elevation {p[2]:.1f} m, heading '
            f'{math.degrees(p[5]):.0f} deg, roll {math.degrees(p[3]):.1f} deg, '
            f'pitch {math.degrees(p[4]):.1f} deg')


def _drive_loop(target, timeout_s, speed, distance=None):
    """Drive toward target (x, y) with proportional steering, or with
    target=None straight ahead (reversing if speed < 0) for `distance` m."""
    start = _pose()
    t0 = time.time()
    while True:
        p = _pose()
        if _tilt_deg(p) > TILT_LIMIT_DEG:
            _halt()
            return f'STOPPED for safety: tilt {_tilt_deg(p):.0f} deg. Now at {_status_line(p)}.'
        if time.time() - t0 > timeout_s:
            _halt()
            return f'STOPPED: timed out after {timeout_s:.0f} s. Now at {_status_line(p)}.'
        if target is None:
            done = math.hypot(p[0] - start[0], p[1] - start[1])
            if done >= distance:
                _halt()
                return f'Done: drove {done:.1f} m. Now at {_status_line(p)}.'
            _send(linear=speed)
        else:
            dx, dy = target[0] - p[0], target[1] - p[1]
            dist = math.hypot(dx, dy)
            if dist < ARRIVED_M:
                _halt()
                return f'Arrived within {dist:.1f} m of the target. Now at {_status_line(p)}.'
            err = math.atan2(math.sin(math.atan2(dy, dx) - p[5]), math.cos(math.atan2(dy, dx) - p[5]))
            if abs(err) > math.radians(60):          # facing well away: turn on the spot first
                _send(rotate=math.copysign(ROTATE_RATE, err))
            else:
                _send(linear=speed, steer=max(-0.6, min(0.6, 1.5 * err)))
        time.sleep(0.1)


@tool
def rover_status() -> str:
    """Current rover position (m, world frame), elevation, heading (deg,
    counter-clockwise from +x/east), roll and pitch, and the nearest of
    Perseverance's waypoints on this world."""
    p = _pose()
    line = 'Rover: ' + _status_line(p) + '.'
    wps = _waypoints()
    if wps:
        w = min(wps, key=lambda w: math.hypot(w['x'] - p[0], w['y'] - p[1]))
        line += f" Nearest waypoint: sol {w['sol']}, {math.hypot(w['x'] - p[0], w['y'] - p[1]):.1f} m away."
    return line


@tool
def list_waypoints() -> str:
    """Perseverance's real waypoints on this world (sol numbers and positions),
    the places the rover can be sent to with drive_to_waypoint."""
    wps = _waypoints()
    if not wps:
        return f'World {WORLD!r} has no waypoint list.'
    p = _pose()
    return '\n'.join(f"sol {w['sol']}: x={w['x']:.1f}, y={w['y']:.1f} "
                     f"({math.hypot(w['x'] - p[0], w['y'] - p[1]):.0f} m from the rover)" for w in wps)


@tool
def drive_distance(meters: float) -> str:
    """Drive straight ahead (negative: reverse) by about this many meters at
    0.3 m/s, then stop. Stops early if the rover tilts too far."""
    if not -30 <= meters <= 30:
        return 'Refused: drive at most 30 m per command.'
    return _drive_loop(None, timeout_s=abs(meters) / SPEED * 2 + 5,
                       speed=math.copysign(SPEED, meters), distance=abs(meters))


@tool
def rotate_in_place(degrees: float) -> str:
    """Rotate on the spot by this many degrees (positive: counter-clockwise /
    left), using the OSR's corner-wheel rotate mode."""
    p0 = _pose()
    target = p0[5] + math.radians(degrees)
    # The pose lags the wheels slightly, so stop, let it settle, and correct
    # what's left; a few rounds get within a few degrees.
    for _ in range(4):
        t0 = time.time()
        err = math.pi
        while time.time() - t0 < abs(math.radians(degrees)) / ROTATE_RATE * 2 + 5:
            p = _pose()
            err = math.atan2(math.sin(target - p[5]), math.cos(target - p[5]))
            if abs(err) < math.radians(3):
                break
            rate = ROTATE_RATE if abs(err) > math.radians(20) else ROTATE_RATE / 2
            _send(rotate=math.copysign(rate, err))
            time.sleep(0.1)
        _halt()
        time.sleep(0.5)
        p = _pose()
        if abs(math.atan2(math.sin(target - p[5]), math.cos(target - p[5]))) < math.radians(5):
            break
    return 'Done. Now at ' + _status_line(_pose()) + '.'


@tool
def drive_to(x: float, y: float) -> str:
    """Drive to a point (x, y in meters, world frame), steering toward it and
    turning on the spot first if it is behind. Stops on arrival (within 1.5 m),
    on too much tilt, or on timeout. Simple go-to-goal: it does not plan
    around rocks."""
    p = _pose()
    dist = math.hypot(x - p[0], y - p[1])
    if dist > 150:
        return f'Refused: target is {dist:.0f} m away; send closer goals (at most 150 m).'
    return _drive_loop((x, y), timeout_s=dist / SPEED * 2.5 + 20, speed=SPEED)


@tool
def drive_to_waypoint(sol: int) -> str:
    """Drive to Perseverance's waypoint for this sol (see list_waypoints)."""
    for w in _waypoints():
        if w['sol'] == sol:
            return drive_to.invoke({'x': w['x'], 'y': w['y']})
    return f'No waypoint for sol {sol} on this world.'


@tool
def stop() -> str:
    """Stop the rover immediately."""
    _halt()
    return 'Stopped.'


TOOLS = [rover_status, list_waypoints, drive_distance, rotate_in_place, drive_to,
         drive_to_waypoint, stop]


def main():
    from langchain_anthropic import ChatAnthropic
    from rosa import ROSA, RobotSystemPrompts
    if not os.environ.get('ANTHROPIC_API_KEY'):
        sys.exit('Set ANTHROPIC_API_KEY.')
    llm = ChatAnthropic(model=os.environ.get('ROSA_MODEL', 'claude-sonnet-5'),
                        temperature=0, max_tokens=4096)
    prompts = RobotSystemPrompts(
        embodiment_and_persona=(
            "You are the NASA-JPL Open Source Rover (OSR), a six-wheeled rocker-bogie "
            "rover, simulated in Gazebo Harmonic under ROS 2 Jazzy."),
        about_your_environment=(
            f"You are on world '{WORLD}': real HiRISE terrain of Jezero Crater, Mars, "
            "with Perseverance's actual waypoints marked by blue posts. Coordinates are "
            "meters in the world frame (x east, y north); headings are degrees "
            "counter-clockwise from east."),
        about_your_capabilities=(
            "Rover tools: rover_status, list_waypoints, drive_distance, rotate_in_place, "
            "drive_to, drive_to_waypoint, stop. ROS tools let you inspect topics, nodes, "
            "services, parameters and logs. Driving commands block until the move ends."),
        constraints_and_guardrails=(
            "Check rover_status before and after moving. Never exceed 30 m per "
            "drive_distance. If a drive stops for safety (tilt), do not repeat the same "
            "move: report it and suggest backing off or another approach."),
        critical_instructions=(
            "Report outcomes honestly using the tools' results, including stops and "
            "timeouts."),
    )
    agent = ROSA(ros_version=2, llm=llm, tools=TOOLS, prompts=prompts, streaming=False)
    print(f"OSR + ROSA on {WORLD}. Ask in plain language; 'exit' quits.")
    while True:
        try:
            q = input('\nyou> ').strip()
        except EOFError:
            break
        if q.lower() in ('exit', 'quit'):
            break
        if q:
            print('\nrover> ' + agent.invoke(q))
    _halt()


if __name__ == '__main__':
    main()
