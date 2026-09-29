"""Spawn the OSR in Gazebo Classic (Humble) or Gazebo Harmonic (Jazzy).

    ros2 launch osr_gz sim.launch.py [sim:=classic|harmonic] [collision:=primitive|mesh]
                                     [gui:=true|false] [world:=<sdf>]
                                     [realtime:=true|false [step:=0.005]]

sim defaults from ROS_DISTRO (humble -> classic, anything newer -> harmonic).
Drives with the same osr_control rover node and kinematics as the hardware stack,
via gazebo_command_adapter, as upstream osr_gazebo does.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def default_sim():
    return 'classic' if os.environ.get('ROS_DISTRO') == 'humble' else 'harmonic'


def strip_comments(node):
    for child in list(node.childNodes):
        if child.nodeType == child.COMMENT_NODE:
            node.removeChild(child)
        else:
            strip_comments(child)


def unthrottled_world(harmonic, step):
    """The packaged unthrottled world with its physics step set to `step`
    seconds, written to a temp file. RTF scales ~linearly with step size."""
    src = os.path.join(get_package_share_directory('osr_gz'), 'worlds',
                       'empty_unthrottled.' + ('sdf' if harmonic else 'world'))
    with open(src) as f:
        sdf = f.read()
    tag = '<max_step_size>0.001</max_step_size>'
    if tag not in sdf:
        raise RuntimeError(f'{src}: expected {tag}')
    fd, path = tempfile.mkstemp(prefix='osr_gz_', suffix=os.path.splitext(src)[1])
    with os.fdopen(fd, 'w') as f:
        f.write(sdf.replace(tag, f'<max_step_size>{step}</max_step_size>'))
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
    if not realtime and not world:
        world = unthrottled_world(harmonic, float(LaunchConfiguration('step').perform(context)))
    # Nodes that stamp or schedule by time run on sim time. The rover node and
    # command adapter only translate commands and never read the clock, so they
    # stay on wall time (as upstream runs them): on Harmonic the bridge
    # publishes /clock on every physics step, and a Python node on sim time
    # burns a full core just receiving it when the sim runs unthrottled.
    sim_time = {'use_sim_time': True}

    xacro_file = os.path.join(get_package_share_directory('osr_gz'), 'urdf', 'osr.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'sim': sim, 'collision': collision})
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
                        {'enable_odometry': False, 'publish_transform': False}],
        ),
        Node(
            package='osr_gz',
            executable='gazebo_command_adapter.py',
            name='gazebo_command_adapter',
            output='screen',
        ),
    ]

    if harmonic:
        gz_args = f"-r {world or 'empty.sdf'}"
        if not gui:
            gz_args = '-s ' + gz_args
        actions += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
                launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', 'robot_description', '-name', 'rover', '-z', '0.05'],
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
        classic_args = {'gui': 'true' if gui else 'false'}
        if world:
            classic_args['world'] = world
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
        DeclareLaunchArgument('world', default_value='',
                              description='world file; empty uses the simulator default'),
        DeclareLaunchArgument('step', default_value='0.005',
                              description='realtime:=false only: physics step [s]. '
                                          '0.005 divides the 100 Hz controller period'),
        DeclareLaunchArgument('realtime', default_value='true',
                              description='false: empty world stepping as fast as the '
                                          'CPU allows (ignored when world is set)'),
        OpaqueFunction(function=launch_setup),
    ])
