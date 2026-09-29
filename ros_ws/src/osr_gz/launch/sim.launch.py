"""Spawn the OSR in Gazebo Classic (Humble) or Gazebo Harmonic (Jazzy).

    ros2 launch osr_gz sim.launch.py [sim:=classic|harmonic] [collision:=primitive|mesh]
                                     [gui:=true|false] [world:=<name or path>]
                                     [realtime:=true|false [step:=0.005]]

world:=jezero_delta (Harmonic) loads worlds/jezero_delta.sdf and spawns the rover
at the start pose in worlds/jezero_delta.yaml. With no world, an empty world.

sim defaults from ROS_DISTRO (humble -> classic, anything newer -> harmonic).
Drives with the same osr_control rover node and kinematics as the hardware stack,
via gazebo_command_adapter, as upstream osr_gazebo does.
"""
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml

# Simulated wheel radius (wheel.stl). osr_params.yaml ships 0.075 for the
# hardware; upstream's rover_bringup.md shows 0.082 as the override. Without it
# every sim runs ~9% faster than commanded.
SIM_WHEEL_RADIUS = 0.082


def default_sim():
    return 'classic' if os.environ.get('ROS_DISTRO') == 'humble' else 'harmonic'


def strip_comments(node):
    for child in list(node.childNodes):
        if child.nodeType == child.COMMENT_NODE:
            node.removeChild(child)
        else:
            strip_comments(child)


def resolve_world(world, harmonic):
    """A world name (looked up in osr_gz/worlds) or a path. Returns (path, meta)
    where meta is the world's .yaml (spawn pose, waypoints) if it has one."""
    worlds = os.path.join(get_package_share_directory('osr_gz'), 'worlds')
    if not world:
        world = 'empty'
    if os.sep not in world and not os.path.splitext(world)[1]:
        world = os.path.join(worlds, world + ('.sdf' if harmonic else '.world'))
    if not os.path.exists(world):
        raise RuntimeError(f'world not found: {world}')
    meta_path = os.path.splitext(world)[0] + '.yaml'
    meta = yaml.safe_load(open(meta_path)) if os.path.exists(meta_path) else {}
    return world, meta


def configure_physics(world, harmonic, realtime, step):
    """Copy `world` to a temp file with its physics set for real time (1 ms
    step, held to wall clock) or unthrottled (`step` seconds, as fast as the
    CPU allows; RTF scales ~linearly with step)."""
    with open(world) as f:
        sdf = f.read()
    step = 0.001 if realtime else step
    sdf, n = re.subn(r'<max_step_size>[^<]*</max_step_size>',
                     f'<max_step_size>{step}</max_step_size>', sdf)
    if harmonic:
        sdf, m = re.subn(r'<real_time_factor>[^<]*</real_time_factor>',
                         f'<real_time_factor>{1 if realtime else 0}</real_time_factor>', sdf)
    else:
        sdf, m = re.subn(r'<real_time_update_rate>[^<]*</real_time_update_rate>',
                         f'<real_time_update_rate>{1000 if realtime else 0}</real_time_update_rate>',
                         sdf)
    if n != 1 or m != 1:
        raise RuntimeError(f'{world}: expected one <physics> block with a step size '
                           'and real-time setting')
    fd, path = tempfile.mkstemp(prefix='osr_gz_', suffix=os.path.splitext(world)[1])
    with os.fdopen(fd, 'w') as f:
        f.write(sdf)
    return path


def launch_setup(context):
    sim = LaunchConfiguration('sim').perform(context)
    collision = LaunchConfiguration('collision').perform(context)
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    world = LaunchConfiguration('world').perform(context)
    realtime = LaunchConfiguration('realtime').perform(context) == 'true'
    if sim not in ('classic', 'harmonic'):
        raise RuntimeError(f"sim must be 'classic' or 'harmonic', got {sim!r}")

    harmonic = sim == 'harmonic'
    world, meta = resolve_world(world, harmonic)
    world = configure_physics(world, harmonic, realtime,
                              float(LaunchConfiguration('step').perform(context)))
    spawn = meta.get('spawn', {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0})
    # Nodes that stamp or schedule by time run on sim time. The rover node and
    # command adapter only translate commands and never read the clock, so they
    # stay on wall time (as upstream runs them): on Harmonic the bridge
    # publishes /clock on every physics step, and a Python node on sim time
    # burns a full core just receiving it when the sim runs unthrottled.
    sim_time = {'use_sim_time': True}

    xacro_file = os.path.join(get_package_share_directory('osr_gz'), 'urdf', 'osr.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={
        'sim': sim, 'collision': collision,
        'wheel_mu': LaunchConfiguration('wheel_mu').perform(context)})
    # gazebo_ros2_control (Humble) passes robot_description to rcl as a
    # `--param` command-line override, and rcl fails to parse it when XML
    # comments are present. Comments aren't needed at runtime, so drop them.
    strip_comments(doc)
    robot_description = doc.toxml()

    osr_params = os.path.join(
        get_package_share_directory('osr_bringup'), 'config', 'osr_params.yaml')

    actions = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}, sim_time],
        ),
        Node(
            package='osr_control',
            executable='rover',
            name='rover',
            output='screen',
            parameters=[osr_params,
                        {'enable_odometry': False, 'publish_transform': False,
                         'rover_dimensions.wheel_radius': SIM_WHEEL_RADIUS}],
        ),
        Node(
            package='osr_gz',
            executable='gazebo_command_adapter.py',
            name='gazebo_command_adapter',
            output='screen',
        ),
    ]

    if harmonic:
        gz_args = f'-r {world}'
        if not gui:
            gz_args = '-s ' + gz_args
        models = os.path.join(get_package_share_directory('osr_gz'), 'models')
        actions += [
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(
                p for p in (models, os.environ.get('GZ_SIM_RESOURCE_PATH')) if p)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
                launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                # Drop from 0.3 m above the ground so the wheels settle onto
                # the terrain rather than spawning intersecting it.
                arguments=['-topic', 'robot_description', '-name', 'rover',
                           '-x', str(spawn['x']), '-y', str(spawn['y']),
                           '-z', str(spawn['z'] + 0.3), '-Y', str(spawn['yaw'])],
                output='screen',
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                           '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
                output='screen',
            ),
        ]
    else:
        classic_args = {'gui': 'true' if gui else 'false', 'world': world}
        actions += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
                launch_arguments=classic_args.items(),
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'rover'],
                output='screen',
            ),
        ]

    # The spawners wait for /controller_manager, which appears once the robot is
    # spawned and the ros2_control plugin has loaded.
    actions += [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager-timeout', '60'],
            parameters=[sim_time],
            output='screen',
        )
        for name in ('joint_state_broadcaster', 'wheel_controller', 'servo_controller')
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value=default_sim(),
                              description='classic (Gazebo 11) or harmonic (gz-sim 8)'),
        DeclareLaunchArgument('collision', default_value='primitive',
                              description='primitive (fast) or mesh (upstream)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('wheel_mu', default_value='0.7',
                              description='wheel-ground friction coefficient'),
        DeclareLaunchArgument('world', default_value='',
                              description='world name in osr_gz/worlds (e.g. jezero_delta) '
                                          'or a path; empty: an empty world'),
        DeclareLaunchArgument('step', default_value='0.005',
                              description='realtime:=false only: physics step [s]. '
                                          '0.005 divides the 100 Hz controller period'),
        DeclareLaunchArgument('realtime', default_value='true',
                              description='false: step as fast as the CPU allows'),
        OpaqueFunction(function=launch_setup),
    ])
