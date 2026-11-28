"""
Pilsbot in Gazebo Fortress, driven by the robot's own stack.

Starts gz sim (ros_gz_sim) with a world, spawns the robot from
robot_description and starts pilsbot_bringup's pilsbot.launch.py with
hardware:=gazebo use_sim_time:=true: the same robot_state_publisher,
spawners, hardware activator, teleop and foxglove_bridge as on the robot. The
controller manager runs inside Gazebo's gz_ros2_control plugin.

Bridges to ROS: /clock and camera_info through parameter_bridge
(config/bridge.yaml), the images through ros_gz_image's image_bridge
(image_transport, so a compressed transport appears beside each), and the
OAK's left, right and depth camera_info through scripts/camera_info_relay.py,
which fixes the stereo baseline. cameras:=false leaves the camera sensors
out of the URDF and starts neither image_bridge nor the relay.

world:= takes a name (flat, citysim, citysim_block) or an SDF path. Each
named world has its spawn point: citysim's is on a road. x, y, z and yaw
override it one by one.

Arguments of pilsbot_bringup (e.g. foxglove:=false) pass through.
"""

import os
import shlex

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, \
    OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='flat',
            description='flat (flat ground), citysim (the full city), citysim_block '
                        '(a block of it, faster), or an SDF file.'),
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description='Start the Gazebo GUI. false: server only, cameras render headless.'),
        DeclareLaunchArgument(
            'robot_profile', default_value='pilsbot',
            description='Robot profile: a name shipped with pilsbot_description, or a path.'),
        *[DeclareLaunchArgument(
            name, default_value='',
            description=f"{description}. Default: the world's spawn point "
                        f"(flat or a path: {FLAT_SPAWN[name]}; citysim: {CITYSIM_SPAWN[name]}).")
          for name, description in [('x', 'Spawn position [m]'), ('y', 'Spawn position [m]'),
                                    ('z', 'Spawn height [m]'), ('yaw', 'Spawn heading [rad]')]],
        DeclareLaunchArgument(
            'cameras', default_value='true', choices=['true', 'false'],
            description='false: no camera sensors in Gazebo and no camera topics '
                        '(nothing renders, control tests run faster).'),
        OpaqueFunction(function=launch_setup),
    ])


# Spawn points. flat: the origin, just above the ground. citysim: on road_x_3,
# right lane, heading along the road (+x), just above the street (5.01 m);
# citysim_block is cropped around it (citysim's README).
FLAT_SPAWN = {'x': '0.0', 'y': '0.0', 'z': '0.02', 'yaw': '0.0'}
CITYSIM_SPAWN = {'x': '-35.0', 'y': '-1.8', 'z': '5.03', 'yaw': '0.0'}

# world:= names: (package, SDF file in its share directory, spawn point).
WORLDS = {
    'flat': ('pilsbot_simulation', 'worlds/flat.sdf', FLAT_SPAWN),
    'citysim': ('citysim', 'worlds/simple_city.sdf', CITYSIM_SPAWN),
    'citysim_block': ('citysim', 'worlds/simple_city_block.sdf', CITYSIM_SPAWN),
}


def resolve_world(world):
    """(SDF file for gz sim, spawn point) of a world:= value."""
    if world in WORLDS:
        package, sdf, spawn = WORLDS[world]
        try:
            share = get_package_share_directory(package)
        except PackageNotFoundError:
            raise RuntimeError(
                f'world:="{world}" needs the {package} package, which is not installed '
                '(or not sourced).') from None
        return os.path.join(share, sdf), spawn
    # A path, or a file name that gz sim looks up on its resource path.
    if '/' in world or os.path.splitext(world)[1] in ('.sdf', '.world'):
        return world, FLAT_SPAWN
    raise RuntimeError(
        f'world:="{world}" is neither a world name ({", ".join(WORLDS)}) nor a path to an '
        'SDF file (a path contains "/" or ends with .sdf or .world).')


# The robot's image topics (gz and ROS use the same name), and the compressed
# transport the robot offers on each besides raw. Depth is 32FC1 in the sim:
# compressed (JPEG/PNG) fails on every frame, compressedDepth works.
IMAGE_TRANSPORTS = {
    '/camera/color/image_raw': 'compressed',
    '/camera/left/image_rect': 'compressed',
    '/camera/right/image_rect': 'compressed',
    '/camera/stereo/depth': 'compressedDepth',
    '/front_camera/image_raw': 'compressed',
}


def image_transport_plugins():
    """image_transport parameters: raw and the robot's compressed transport per image."""
    # <topic without the leading slash, dots for slashes>.enable_pub_plugins.
    # Without it every installed plugin (theora too) publishes.
    return {f"{topic.lstrip('/').replace('/', '.')}.enable_pub_plugins":
            ['image_transport/raw', f'image_transport/{transport}']
            for topic, transport in IMAGE_TRANSPORTS.items()}


def launch_setup(context):
    cameras = LaunchConfiguration('cameras').perform(context) == 'true'
    world, spawn_point = resolve_world(LaunchConfiguration('world').perform(context))
    spawn_pose = {name: LaunchConfiguration(name).perform(context) or default
                  for name, default in spawn_point.items()}

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
                   '-x', spawn_pose['x'], '-y', spawn_pose['y'],
                   '-z', spawn_pose['z'], '-Y', spawn_pose['yaw']])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution(
                [FindPackageShare('pilsbot_simulation'), 'config',
                 'bridge.yaml' if cameras else 'bridge_no_cameras.yaml']),
            'use_sim_time': True,
        }])

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('pilsbot_bringup'), '/launch/pilsbot.launch.py']),
        launch_arguments={
            'hardware': 'gazebo',
            'use_sim_time': 'true',
            'robot_profile': LaunchConfiguration('robot_profile'),
            'cameras': LaunchConfiguration('cameras'),
        }.items())

    if not cameras:
        return [gazebo, spawn, bridge, robot]

    image_bridge = Node(
        package='ros_gz_image', executable='image_bridge', output='screen',
        arguments=list(IMAGE_TRANSPORTS),
        parameters=[{'use_sim_time': True, **image_transport_plugins()}])

    camera_info_relay = Node(
        package='pilsbot_simulation', executable='camera_info_relay.py', output='screen',
        parameters=[{'use_sim_time': True}])

    return [gazebo, spawn, bridge, image_bridge, camera_info_relay, robot]
