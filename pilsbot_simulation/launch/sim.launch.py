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

import os
import shlex

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, \
    OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration
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
        # SPIKE (pilsbot_simulation#4)
        DeclareLaunchArgument(
            'cameras', default_value='true', choices=['true', 'false'],
            description='SPIKE: false leaves the camera sensors out (PILSBOT_SIM_CAMERAS).'),
        DeclareLaunchArgument(
            'images', default_value='parameter_bridge',
            choices=['parameter_bridge', 'image_bridge'],
            description='SPIKE: bridge the images with parameter_bridge (M1) or '
                        'ros_gz_image image_bridge (image_transport, /compressed).'),
        OpaqueFunction(function=launch_setup),
    ])


IMAGE_TOPICS = ['/camera/color/image_raw', '/camera/left/image_rect',
                '/camera/right/image_rect', '/camera/stereo/depth', '/front_camera/image_raw']


def launch_setup(context):
    world = LaunchConfiguration('world').perform(context)
    if world == 'citysim':
        world = os.path.join(get_package_share_directory('citysim'), 'worlds', 'simple_city.sdf')
    elif world == 'citysim_block':
        world = os.path.join(get_package_share_directory('citysim'), 'worlds',
                             'simple_city_block.sdf')
    elif world == 'frame_test':
        world = os.path.join(get_package_share_directory('pilsbot_simulation'), 'worlds',
                             'frame_test.sdf')
    image_bridge = LaunchConfiguration('images').perform(context) == 'image_bridge'
    gz_args = ['-r', '-v', '3']
    if LaunchConfiguration('gui').perform(context) == 'false':
        gz_args = ['-s', '--headless-rendering'] + gz_args
    gz_args.append(world)

    # ros_gz_sim runs `ruby ign gazebo <gz_args>` through a shell (shell=True):
    # quote the arguments, and exec, so that the shell is replaced by Gazebo.
    # Otherwise launch's SIGINT/SIGTERM on shutdown stops the shell only and
    # the gz server outlives the launch. The scoped group keeps the prefix
    # away from the robot's nodes.
    gazebo = GroupAction(scoped=True, actions=[
        SetLaunchConfiguration('launch-prefix', 'exec'),
        SetLaunchConfiguration('launch-prefix-filter', ''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
            launch_arguments={
                'gz_args': ' '.join(shlex.quote(arg) for arg in gz_args),
                # Without Gazebo there is no controller manager: end everything.
                'on_exit_shutdown': 'true',
            }.items()),
    ])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'pilsbot',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution(
                [FindPackageShare('pilsbot_simulation'), 'config',
                 'bridge_no_images.yaml' if image_bridge else 'bridge.yaml']),
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

    actions = [SetEnvironmentVariable('PILSBOT_SIM_CAMERAS', LaunchConfiguration('cameras')),
               gazebo, spawn, bridge, robot]
    if image_bridge:
        actions.append(Node(
            package='ros_gz_image', executable='image_bridge', output='screen',
            arguments=IMAGE_TOPICS, parameters=[{'use_sim_time': True}]))
    return actions
