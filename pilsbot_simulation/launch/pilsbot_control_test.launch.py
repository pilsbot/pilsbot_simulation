import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), '/launch/gazebo.launch.py']),
    )

    pkg_share = FindPackageShare('pilsbot_description').find('pilsbot_description')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    xacro_file = os.path.join(urdf_dir, 'pilsbot.urdf.xacro')
    robot_desc = Command('xacro {}'.format(xacro_file))

    pilsbot_drive_controller = PathJoinSubstitution(
        [
            FindPackageShare("pilsbot_control"),
            "config",
            "acker_diff_controller.yaml",
        ]
    )

    params = {'robot_description': robot_desc,}
              #'use_sim_time': use_sim_time}

    robot_description = Node(package='robot_state_publisher',
                             executable='robot_state_publisher',
                             output='both',
                             parameters=[params])



    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description', '-entity', 'pilsbot'],
                        output='screen')

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # name="pilsbot_controller_manager",
        parameters=[params, pilsbot_drive_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_dd_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["pilsbot_velocity_controller"],
        output="screen",
    )
    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["pilsbot_joint_publisher"],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        robot_description,
        spawn_entity,
        controller_manager_node,
        spawn_dd_controller,
        spawn_jsb_controller,
    ])
