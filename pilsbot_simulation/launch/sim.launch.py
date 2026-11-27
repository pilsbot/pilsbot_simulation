"""SPIKE pilsbot/pilsbot_simulation#1: Pilsbot on Gazebo Fortress.

Throwaway. Starts gz sim, spawns the robot from robot_description, bridges
/clock, the OAK sensors and ground truth, and runs the robot's own start
sequence (spawners --inactive + hardware_activator) against the controller
manager inside gz_ros2_control. pilsbot_bringup is not included yet (that is
the job of the later tickets).
"""

import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from pilsbot_description import profile

HARDWARE_COMPONENT = 'PilsbotSystem'


def generate_launch_description():
    args = [
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution(
            [FindPackageShare('pilsbot_simulation'), 'worlds', 'flat.sdf'])),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('robot_profile', default_value='pilsbot'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.02'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # spike knobs
        DeclareLaunchArgument('pivot_damping', default_value='0.0'),
        DeclareLaunchArgument('pivot_friction', default_value='0.0'),
        DeclareLaunchArgument('wheel_mu', default_value='1.0'),
        DeclareLaunchArgument('wheel_inertia', default_value='placeholder',
                              description='placeholder (1.0 kg m^2) | solid'),
        DeclareLaunchArgument('physics_engine', default_value='',
                              description='e.g. ignition-physics-bullet-plugin'),
        DeclareLaunchArgument('activator', default_value='true'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    lc = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    robot_profile = profile.load(lc('robot_profile'))

    overrides_params = dict(robot_profile.controller_overrides())
    overrides_params['use_sim_time'] = True
    with tempfile.NamedTemporaryFile(
            'w', prefix='pilsbot_sim_overrides_', suffix='.yaml', delete=False) as overrides:
        yaml.safe_dump({
            'controller_manager': {'ros__parameters': {'use_sim_time': True}},
            'joint_state_publisher': {'ros__parameters': {'use_sim_time': True}},
            'pilsbot_velocity_controller': {'ros__parameters': overrides_params},
        }, overrides)

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('pilsbot_description'), 'urdf', 'pilsbot.urdf.xacro']),
        ' hardware:=gazebo',
        f' controller_overrides:={overrides.name}',
        f' robot_profile:={robot_profile.path}',
        f' pivot_damping:={lc("pivot_damping")}',
        f' pivot_friction:={lc("pivot_friction")}',
        f' wheel_mu:={lc("wheel_mu")}',
        f' wheel_inertia:={lc("wheel_inertia")}',
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)}

    gz_args = f'-r -v 3 {lc("world")}'
    if lc('gui').lower() not in ('true', '1'):
        gz_args = '-s --headless-rendering ' + gz_args
    if lc('physics_engine'):
        gz_args = f'--physics-engine {lc("physics_engine")} ' + gz_args

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='both', parameters=[robot_description, {'use_sim_time': True}])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'pilsbot',
                            '-x', lc('x'), '-y', lc('y'), '-z', lc('z'), '-Y', lc('yaw')])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': PathJoinSubstitution(
                      [FindPackageShare('pilsbot_simulation'), 'config', 'bridge.yaml']),
                      'use_sim_time': True}])

    actions = [gz, rsp, spawn, bridge]

    # The robot's start sequence (ADR 0007), unchanged.
    actions += [
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['pilsbot_velocity_controller', '--inactive']),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['joint_state_publisher', '--inactive']),
    ]
    if lc('activator').lower() in ('true', '1'):
        actions.append(Node(
            package='pilsbot_driver', executable='hardware_activator', output='screen',
            parameters=[{'hardware_component': HARDWARE_COMPONENT,
                         'controllers': ['joint_state_publisher', 'pilsbot_velocity_controller'],
                         'use_sim_time': True}]))
    return actions
