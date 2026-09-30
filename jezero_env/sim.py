"""In-process Gazebo Harmonic simulation of the OSR, driven step by step.

No ROS in the loop: the gz-sim server runs inside this Python process
(gz.sim8.TestFixture), commands are applied to the joints from a pre-update
callback on every physics step, and `step(n)` returns after exactly n physics
steps. That makes a step deterministic, with no transport latency between an
action and the physics seeing it.

Commands go through the rover's own inverse kinematics (osr_control.kinematics,
pure Python) and the same sign conventions as upstream's gazebo_command_adapter,
so a twist here drives the wheels exactly as /cmd_vel does in the ROS stack.

Needs the system gz-sim (see run.sh for the environment).
"""
from __future__ import annotations

import math
import os
import re
import itertools
import subprocess
import tempfile
import time
from dataclasses import dataclass

import yaml
import gz.math7  # noqa: F401  gz.sim8 returns gz.math7 types (Pose3d, Vector3d); pybind needs them registered
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.sim8 import Joint, Link, Model, TestFixture, World, world_entity
from gz.transport13 import Node
from osr_control.kinematics import RoverDimensions, RoverKinematics

OSR_GZ = os.environ.get('OSR_GZ_SHARE', '/osr_ws/install/osr_gz/share/osr_gz')
OSR_PARAMS = os.environ.get(
    'OSR_PARAMS', '/osr_ws/install/osr_bringup/share/osr_bringup/config/osr_params.yaml')

# Same as sim.launch.py: the simulated wheel (wheel.stl), not the hardware default.
SIM_WHEEL_RADIUS = 0.082

# Joint order and signs from upstream gazebo_command_adapter.py: right-side drive
# velocities and all corner angles are negated for Gazebo's joint axes.
WHEELS = (  # (joint, DriveCommand field, sign)
    ('front_wheel_joint_left', 'left_front_vel', 1.0),
    ('middle_wheel_joint_left', 'left_middle_vel', 1.0),
    ('rear_wheel_joint_left', 'left_back_vel', 1.0),
    ('front_wheel_joint_right', 'right_front_vel', -1.0),
    ('middle_wheel_joint_right', 'right_middle_vel', -1.0),
    ('rear_wheel_joint_right', 'right_back_vel', -1.0),
)
CORNERS = (  # (joint, CornerCommand field, sign)
    ('front_wheel_joint_L', 'left_front_pos', -1.0),
    ('rear_wheel_joint_L', 'left_back_pos', -1.0),
    ('front_wheel_joint_R', 'right_front_pos', -1.0),
    ('rear_wheel_joint_R', 'right_back_pos', -1.0),
)
# Corner servo: proportional position control, as a velocity command each step.
CORNER_KP = 10.0        # 1/s
CORNER_MAX_VEL = 3.0    # rad/s


def rover_kinematics() -> RoverKinematics:
    params = yaml.safe_load(open(OSR_PARAMS))['rover']['ros__parameters']
    d = params['rover_dimensions']
    dims = RoverDimensions(d1=d['d1'], d2=d['d2'], d3=d['d3'], d4=d['d4'],
                           wheel_radius=SIM_WHEEL_RADIUS)
    return RoverKinematics(dims, drive_no_load_rpm=params['drive_no_load_rpm'])


def build_rover_model(out_dir: str, collision: str = 'primitive', wheel_mu: float = 0.7,
                      camera: bool = False) -> str:
    """Rover SDF model dir from the osr_gz xacro, with no simulator plugins
    (sim:=gym). Runs xacro and `gz sdf` in a ROS-sourced shell, once."""
    model_dir = os.path.join(out_dir, 'osr_rover')
    os.makedirs(model_dir, exist_ok=True)
    cmd = (
        'source /opt/ros/jazzy/setup.bash && source /osr_ws/install/setup.bash && '
        f'xacro {OSR_GZ}/urdf/osr.urdf.xacro sim:=gym collision:={collision} '
        f'wheel_mu:={wheel_mu} camera:={str(camera).lower()} > {model_dir}/rover.urdf && '
        f'gz sdf -p {model_dir}/rover.urdf'
    )
    # A clean environment: the ROS setup scripts manage LD_LIBRARY_PATH themselves.
    sdf = subprocess.run(['bash', '-c', cmd], check=True, capture_output=True, text=True,
                         env={'PATH': '/usr/bin:/bin', 'HOME': os.environ.get('HOME', '/root')}
                         ).stdout
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
        f.write(sdf)
    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write('<?xml version="1.0"?><model><name>osr_rover</name>'
                '<sdf version="1.9">model.sdf</sdf></model>')
    return model_dir


@dataclass
class RoverState:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    vx: float       # world-frame linear velocity
    vy: float
    vz: float
    wx: float       # world-frame angular velocity
    wy: float
    wz: float
    sim_time: float


_instances = itertools.count()


class JezeroSim:
    """One gz-sim world with the OSR in it.

    world: a name in osr_gz/worlds (e.g. 'jezero_delta', 'empty') or a path.
    step_size: physics step [s]; the world runs unthrottled.
    spawn: (x, y, z, yaw) override; defaults to the world's .yaml spawn pose.
    camera: add the chase camera (topic /chase_cam) and the Sensors system, for
        recordings. Rendering needs a display (see eval/record.py).
    """

    def __init__(self, world: str = 'jezero_delta', step_size: float = 0.005,
                 spawn: tuple | None = None, collision: str = 'primitive',
                 wheel_mu: float = 0.7, camera: bool = False):
        self.step_size = step_size
        self.kin = rover_kinematics()
        # gz-transport is host-wide: two sims (in one process, or across a
        # SubprocVecEnv) would advertise the same /world/<name>/... services and
        # answer each other's requests. A partition per instance isolates them.
        # Read when the server and our Node are created, below.
        self.partition = f'jezero_{os.getpid()}_{next(_instances)}'
        os.environ['GZ_PARTITION'] = self.partition
        self._tmp = tempfile.mkdtemp(prefix='jezero_env_')
        self.camera = camera
        build_rover_model(self._tmp, collision=collision, wheel_mu=wheel_mu, camera=camera)

        world_path = world if os.sep in world else os.path.join(OSR_GZ, 'worlds', world + '.sdf')
        meta_path = os.path.splitext(world_path)[0] + '.yaml'
        self.meta = yaml.safe_load(open(meta_path)) if os.path.exists(meta_path) else {}
        if spawn is None:
            s = self.meta.get('spawn', {'x': 0, 'y': 0, 'z': 0, 'yaw': 0})
            spawn = (s['x'], s['y'], s['z'], s['yaw'])
        self.spawn = spawn

        os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(
            [self._tmp, os.path.join(OSR_GZ, 'models')]
            + [p for p in [os.environ.get('GZ_SIM_RESOURCE_PATH')] if p])
        self._world_file = self._prepare_world(world_path, spawn)

        # Commands, applied every physics step by _pre_update.
        self._wheel_vel = {name: 0.0 for name, _, _ in WHEELS}
        self._corner_target = {name: 0.0 for name, _, _ in CORNERS}
        self._joints = None
        self._link = None
        self._state = None
        self._node = Node()
        self._rover_present = False   # updated every step
        self._resetting = False       # no joint lookup/commands while swapping models

        self.fixture = TestFixture(self._world_file)
        self.fixture.on_pre_update(self._pre_update)
        self.fixture.on_post_update(self._post_update)
        self.fixture.finalize()
        self.server = self.fixture.server()
        self.step(1)  # create entities, populate state

    # -- world ------------------------------------------------------------

    def _prepare_world(self, path, spawn):
        with open(path) as f:
            sdf = f.read()
        sdf, n = re.subn(r'<max_step_size>[^<]*</max_step_size>',
                         f'<max_step_size>{self.step_size}</max_step_size>', sdf)
        sdf, m = re.subn(r'<real_time_factor>[^<]*</real_time_factor>',
                         '<real_time_factor>0</real_time_factor>', sdf)
        if n != 1 or m != 1:
            raise RuntimeError(f'{path}: expected one <physics> block')
        self.world_name = re.search(r'<world name="([^"]+)"', sdf).group(1)
        if self.camera:
            sdf = sdf.replace(
                '<plugin filename="gz-sim-physics-system"',
                '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
                '<render_engine>ogre2</render_engine></plugin>\n    '
                '<plugin filename="gz-sim-physics-system"', 1)
        x, y, z, yaw = spawn
        rover = (f'<include><uri>model://osr_rover</uri><name>rover</name>'
                 f'<pose>{x} {y} {z + 0.3} 0 0 {yaw}</pose></include>\n  </world>')
        sdf = sdf.replace('</world>', rover, 1)
        out = os.path.join(self._tmp, 'world.sdf')
        with open(out, 'w') as f:
            f.write(sdf)
        return out

    # -- callbacks (run inside the physics loop) ----------------------------

    def _lookup(self, ecm):
        """Find the rover's joints and body link. False while no rover exists
        (between remove and create during a reset)."""
        model = Model(World(world_entity(ecm)).model_by_name(ecm, 'rover'))
        if not model.valid(ecm):
            return False
        self._joints = {}
        for name in list(self._wheel_vel) + list(self._corner_target):
            joint = Joint(model.joint_by_name(ecm, name))
            joint.enable_position_check(ecm, True)
            joint.enable_velocity_check(ecm, True)
            self._joints[name] = joint
        self._link = Link(model.link_by_name(ecm, 'base_footprint'))
        self._link.enable_velocity_checks(ecm, True)
        return True

    def _pre_update(self, info, ecm):
        if info.paused or self._resetting:
            return
        if self._joints is None and not self._lookup(ecm):
            return
        for name, vel in self._wheel_vel.items():
            self._joints[name].set_velocity(ecm, [vel])
        for name, target in self._corner_target.items():
            pos = self._joints[name].position(ecm)
            cur = pos[0] if pos else 0.0
            vel = max(-CORNER_MAX_VEL, min(CORNER_MAX_VEL, CORNER_KP * (target - cur)))
            self._joints[name].set_velocity(ecm, [vel])

    def _post_update(self, info, ecm):
        self._rover_present = Model(
            World(world_entity(ecm)).model_by_name(ecm, 'rover')).valid(ecm)
        if self._resetting or self._link is None or not self._joints:
            return
        pose = self._link.world_pose(ecm)
        vel = self._link.world_linear_velocity(ecm)
        ang = self._link.world_angular_velocity(ecm)
        p, q = pose.pos(), pose.rot()
        self._state = RoverState(
            x=p.x(), y=p.y(), z=p.z(), roll=q.roll(), pitch=q.pitch(), yaw=q.yaw(),
            vx=vel.x() if vel else 0.0, vy=vel.y() if vel else 0.0, vz=vel.z() if vel else 0.0,
            wx=ang.x() if ang else 0.0, wy=ang.y() if ang else 0.0,
            wz=ang.z() if ang else 0.0,
            sim_time=info.sim_time.total_seconds())

    # -- public API ---------------------------------------------------------

    def command(self, linear_x: float = 0.0, angular_z: float = 0.0, angular_y: float = 0.0):
        """Set the drive command, as a /cmd_vel Twist would. Same rules as the
        rover node (mathematical mode): angular_y with no linear_x rotates in place."""
        k = self.kin
        if angular_y and not linear_x:
            corner, drive = k.calculate_rotate_in_place_cmd(angular_y)
        else:
            radius = k.twist_to_turning_radius(linear_x, angular_z)
            corner = k.calculate_corner_positions(radius)
            drive = k.calculate_drive_velocities(k.body_speed_for_radius(linear_x, radius), radius)
        for name, field, sign in WHEELS:
            self._wheel_vel[name] = sign * getattr(drive, field)
        for name, field, sign in CORNERS:
            self._corner_target[name] = sign * getattr(corner, field)

    def step(self, n: int = 1) -> RoverState:
        """Advance exactly n physics steps; returns the state after the last one."""
        self.server.run(True, n, False)
        return self._state

    def state(self) -> RoverState:
        return self._state

    def _service(self, name, request, request_type, timeout_ms=5000):
        """Call a world service. The server only answers while it is stepping,
        so step in the background for the duration of the call. A reply only
        means the request was queued; callers wait for the effect."""
        self.server.run(False, 50, False)
        ok, reply = self._node.request(f'/world/{self.world_name}/{name}', request,
                                       request_type, Boolean, timeout_ms)
        while self.server.is_running():
            time.sleep(0.001)
        if not (ok and reply.data):
            raise RuntimeError(f'/world/{self.world_name}/{name} failed')

    def _step_until(self, present: bool, max_steps: int = 500):
        for _ in range(max_steps):
            if self._rover_present == present:
                return
            self.server.run(True, 1, False)
        raise RuntimeError(f"rover {'never appeared' if present else 'was never removed'}")

    def reset(self, spawn: tuple | None = None, settle_max_s: float = 10.0) -> RoverState:
        """Replace the rover with a fresh one at `spawn` (default: the world's
        spawn pose), clear the command, and let it settle until it is at rest
        (see _settle), for at most `settle_max_s` of sim time.

        Deleting and re-creating the model is the only clean reset available:
        - Server.reset_all() (gz-sim 8.15) stops TestFixture's pre-update
          callback from firing, so commands are silently ignored afterwards.
        - Teleporting and zeroing joints leaves the body's momentum (a rover
          reset mid-spin keeps spinning), and Link velocity commands persist
          every step, pinning the body in place.
        A new model starts from the same state every time, to within
        micrometres: the creation lands at a varying point in the background
        service window. A variant that counted the settle schedule from the
        creation iteration made seeded resets bit-exact, but trained less
        reliably (1 of 4 seeds learned vs 4 of 4 with this one; see PLAN.md,
        "Seed variance"), so this is the version kept.
        Sim time keeps running; track episode time from the returned state.
        """
        if spawn is not None:
            self.spawn = spawn
        self.command()
        self._resetting = True
        self._joints = self._link = self._state = None
        self._service('remove', Entity(name='rover', type=Entity.MODEL), Entity)
        self._step_until(present=False)
        factory = EntityFactory(sdf_filename=os.path.join(self._tmp, 'osr_rover', 'model.sdf'),
                                name='rover', allow_renaming=False)
        x, y, z, yaw = self.spawn
        factory.pose.position.x, factory.pose.position.y, factory.pose.position.z = x, y, z + 0.3
        factory.pose.orientation.z, factory.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self._service('create', factory, EntityFactory)
        self._step_until(present=True)
        self._resetting = False
        self._created_at = self._state_time()
        return self._settle(settle_max_s)

    def _state_time(self):
        return self.step(1).sim_time

    def _settle(self, max_s: float, chunk_s: float = 0.1, calm_chunks: int = 5,
                v_tol: float = 0.01, w_tol: float = 0.02) -> RoverState:
        """Step until the body is still: speed < v_tol m/s and angular speed
        < w_tol rad/s for `calm_chunks` consecutive chunks. With fixed rockers
        the body pivots on the undamped bogie joints, and at Mars gravity that
        swing is slow: a fixed 1 s settle left it at 40-46 deg pitch, which the
        env then scored as tipping over on step 1."""
        n = max(1, int(round(chunk_s / self.step_size)))
        calm = 0
        s = self.step(n)
        for _ in range(int(max_s / chunk_s)):
            still = (math.sqrt(s.vx ** 2 + s.vy ** 2 + s.vz ** 2) < v_tol
                     and math.sqrt(s.wx ** 2 + s.wy ** 2 + s.wz ** 2) < w_tol)
            calm = calm + 1 if still else 0
            if calm >= calm_chunks:
                break
            s = self.step(n)
        self.settle_time = s.sim_time - self._created_at
        return s
