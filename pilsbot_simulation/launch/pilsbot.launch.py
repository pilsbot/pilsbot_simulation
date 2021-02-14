import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), '/launch/gazebo.launch.py']),
    )

    pkg_share = FindPackageShare('pilsbot_description').find('pilsbot_description')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    xacro_file = os.path.join(urdf_dir, 'pilsbot.urdf.xacro')
    robot_desc = Command('xacro %s' % xacro_file)

    params = {'robot_description': robot_desc,
              'use_sim_time': True}

    robot_description = Node(package='robot_state_publisher',
                             executable='robot_state_publisher',
                             output='both',
                             parameters=[params])

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description', '-entity', 'pilsbot'],
                        output='screen')

    return LaunchDescription([
        gazebo,
        robot_description,
        spawn_entity,
    ])
