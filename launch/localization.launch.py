from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('delta_threshold', default_value='0.1', description='Threshold for delta'),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
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
            executable='rtk_beardist',
            name='rtk_beardist',
            output='screen',
            parameters=[{'delta_threshold': LaunchConfiguration('delta_threshold')}],
        ),
        Node(
            package='rtk_localization',
            executable='rtk_kalman',
            name='rtk_kalman',
            output='screen',
        ),
        Node(
            package='rtk_localization',
            executable='rtk_odometry',
            name='rtk_odometry',
            output='screen',
        ),
        Node(
            package='rtk_localization',
            executable='rtk_transform',
            name='rtk_transform',
            output='screen',
        ),
    ])
