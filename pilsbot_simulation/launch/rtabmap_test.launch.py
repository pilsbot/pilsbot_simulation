from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'delete_db_on_start', default_value='true'
        ),
        DeclareLaunchArgument(
            'stereo', default_value='true'
        ),
        # Declare more arguments here...
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("rtabmap_launch"), '/launch/rtabmap.launch.py']
            ),
            launch_arguments={
                'delete_db_on_start': LaunchConfiguration('delete_db_on_start'),
                'stereo': LaunchConfiguration('stereo'),
                'depth_topic': '/oak_d/depth/depth/image_raw',
                'rgb_topic': '/oak_d/color/image_raw',
                'left_image_topic': '/oak_d/left/image_raw',
                'right_image_topic': '/oak_d/right/image_raw',
                'left_camera_info_topic': '/oak_d/left/camera_info',
                'right_camera_info_topic': '/oak_d/right/camera_info',
                'frame_id': 'oak-d_frame',
                'approx_sync': 'true',
                'approx_sync_max_interval': "0.001",
                'visual_odometry': 'false',
                'odom_topic': '/odom',
                'odom_frame_id': 'odom_raw',
                'use_sim_time': 'true'
            }.items()
        ),

        # Node(
        #     package='rtabmap_launch',
        #     namespace='rtabmap',
        #     executable='rtabmap.launch.p',
        #     output='screen',
        #     parameters=[{
        #         'delete_db_on_start': LaunchConfiguration('delete_db_on_start'),
        #         'stereo': LaunchConfiguration('stereo'),
        #         'left_image_topic': '/oak_d/left/image_raw',
        #         'right_image_topic': '/oak_d/right/image_raw',
        #         'left_camera_info_topic': '/oak_d/left/camera_info',
        #         'right_camera_info_topic': '/oak_d/right/camera_info',
        #         'frame_id': 'oak-d_frame',
        #         'approx_sync': 'true',
        #         'approx_sync_max_interval': 0.001,
        #         'visual_odometry': 'true',
        #         'odom_frame_id': 'odom_raw',
        #         'use_sim_time': 'true',
        #     }]
        # )
])
