"""
Pilsbot in Gazebo Fortress, driven by the robot's own stack.

Starts gz sim (ros_gz_sim) with a world, spawns the robot from
robot_description, bridges /clock and the sensors (config/bridge.yaml) and
starts pilsbot_bringup's pilsbot.launch.py with hardware:=gazebo
use_sim_time:=true: the same robot_state_publisher, spawners, hardware
activator, teleop and foxglove_bridge as on the robot. The controller manager
runs inside Gazebo's gz_ros2_control plugin.

Arguments of pilsbot_bringup (e.g. foxglove:=false) pass through.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution(
                [FindPackageShare('pilsbot_simulation'), 'worlds', 'flat.sdf']),
            description='SDF world file. Default: flat ground.'),
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description='Start the Gazebo GUI. false: server only, cameras render headless.'),
        DeclareLaunchArgument(
            'robot_profile', default_value='pilsbot',
            description='Robot profile: a name shipped with pilsbot_description, or a path.'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn position [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn position [m]'),
        DeclareLaunchArgument(
            'z', default_value='0.02',
            description='Spawn height [m]: just above the ground.'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn heading [rad]'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    gz_args = ['-r', '-v', '3']
    if LaunchConfiguration('gui').perform(context) == 'false':
        gz_args = ['-s', '--headless-rendering'] + gz_args
    gz_args.append(LaunchConfiguration('world').perform(context))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={
            'gz_args': ' '.join(gz_args),
            # Without Gazebo there is no controller manager: end everything.
            'on_exit_shutdown': 'true',
        }.items())

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'pilsbot',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution(
                [FindPackageShare('pilsbot_simulation'), 'config', 'bridge.yaml']),
            'use_sim_time': True,
        }])

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('pilsbot_bringup'), '/launch/pilsbot.launch.py']),
        launch_arguments={
            'hardware': 'gazebo',
            'use_sim_time': 'true',
            'robot_profile': LaunchConfiguration('robot_profile'),
        }.items())

    return [gazebo, spawn, bridge, robot]
