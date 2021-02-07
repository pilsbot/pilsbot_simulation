import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/gazebo.launch.py']),
    )

    pkg_share = FindPackageShare('pilsbot_description').find('pilsbot_description')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    xacro_file = os.path.join(urdf_dir, 'pilsbot.urdf.xacro')
    robot_desc = launch.substitutions.Command('xacro %s' % xacro_file)
    params = {'robot_description': robot_desc}

    robot_description = Node(package='robot_state_publisher',
                            node_executable='robot_state_publisher',
                            output='both',
                            parameters=[params])

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
                        arguments=['-entity', 'pilsbot', '-stdin', robot_desc],
                        output='screen')

    return LaunchDescription([
        gazebo,
        robot_description,
        spawn_entity,
    ])
