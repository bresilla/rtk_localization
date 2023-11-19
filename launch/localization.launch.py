from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('delta', default_value='0.1', description='Threshold for delta'),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        # ),
        # ExecuteProcess(
        #     cmd=[[
        #         FindExecutable(name='ros2'),
        #         "service call",
        #         "/gps/fix_set",
        #         "example_interfaces/srv/Trigger ",
        #     ]],
        #     shell=True
        # ),
        Node(
            package='rtk_localization',
            executable='cord_convert',
            name='cord_convert',
            output='screen',
        ),
        Node(
            package='rtk_localization',
            executable='gps_fuse',
            name='gps_fuse',
            output='screen',
        ),
        Node(
            package='rtk_localization',
            executable='gps_to_enu',
            name='gps_to_enu',
            output='screen',
        ),
        Node(
            package='rtk_localization',
            executable='odometry',
            name='odometry',
            output='screen',
        ),
        Node(
            package='rtk_localization',
            executable='transform',
            name='transform',
            output='screen',
        ),
    ])
