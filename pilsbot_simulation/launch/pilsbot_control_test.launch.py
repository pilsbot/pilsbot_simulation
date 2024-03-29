from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    declared_launch_args = []

    declared_launch_args.append(DeclareLaunchArgument(
        'controller_config', default_value=TextSubstitution(text='acker_diff_controller_sim.yaml'),
        description='The controller configuration you want to use.'))

    declared_launch_args.append(DeclareLaunchArgument(
        'descritption_file', default_value=TextSubstitution(text='pilsbot.urdf.xacro'),
        description='URDF/XACRO description file with the robot.'))

    declared_launch_args.append(DeclareLaunchArgument(
        'description_package', default_value=TextSubstitution(text='pilsbot_description'),
        description=' Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.'))

    declared_launch_args.append(DeclareLaunchArgument(
        'world', default_value=TextSubstitution(text='')
    ))

    declared_launch_args.append(DeclareLaunchArgument(
        'verbose', default_value=TextSubstitution(text='false'),
        description='Set Gazebo output to verbose.'
    ))

    declared_launch_args.append(DeclareLaunchArgument(
        'z_offset', default_value=TextSubstitution(text='0.15'),
        description='Set a z offset for the spawn position.'))

    declared_launch_args.append(DeclareLaunchArgument(
        'cm_timeout', default_value=TextSubstitution(text='15'),
        description='Set a timeout for the controller manager.'))

    # intialise args
    controller_config = LaunchConfiguration('controller_config')
    description_file = LaunchConfiguration('descritption_file')
    description_package = LaunchConfiguration('description_package')
    world = LaunchConfiguration('world')
    verbose = LaunchConfiguration('verbose')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    z_offset = LaunchConfiguration('z_offset')
    cm_timeout = LaunchConfiguration('cm_timeout')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), '/launch/gazebo.launch.py']),
        launch_arguments={'world': world,
                          'verbose': verbose}.items()
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "urdf",
                    description_file,
                ]
            ),
            " ",
            "controller_config:=",
            controller_config,
            " ",
            "use_fake_hardware:=true",
        ]
    )

    params = {'robot_description': robot_description_content,
              'use_sim_time': use_sim_time}

    robot_description = Node(package='robot_state_publisher',
                             executable='robot_state_publisher',
                             output='both',
                             parameters=[params])

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description',
                                   '-entity', 'pilsbot',
                                   '-timeout', '30',
                                   '-z', z_offset],
                        output='screen')

    spawn_dd_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=['pilsbot_velocity_controller',
                   '--controller-manager-timeout', cm_timeout,
                   #'--ros-args', '--log-level', 'DEBUG'
                ],
        output="screen",
    )
    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["pilsbot_joint_publisher",
        '--controller-manager-timeout', cm_timeout],
        output="screen",
    )

    nodes = [gazebo,
             robot_description,
             spawn_entity,
             spawn_dd_controller,
             spawn_jsb_controller,
             ]

    return LaunchDescription(declared_launch_args + nodes)
