from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def get_ros2_nodes():
    antena_arg = DeclareLaunchArgument('double_antenna', default_value='True', description='double antenna')
    antenna_split = Node(
        condition = UnlessCondition(LaunchConfiguration('double_antenna')),
        package='rtk_localization',
        executable='antenna_split',
        name='antenna_split',
        output='screen',
    )
    cord_convert = Node(
        package='rtk_localization',
        executable='cord_convert',
        name='cord_convert',
        output='screen',
    )
    antenna_fuse = Node(
        package='rtk_localization',
        executable='antenna_fuse',
        name='antenna_fuse',
        output='screen',
    )
    gps_to_enu = Node(
        package='rtk_localization',
        executable='gps_to_enu',
        name='gps_to_enu',
        output='screen',
    )
    odometry = Node(
        package='rtk_localization',
        executable='odometry',
        name='odometry',
        output='screen',
    )
    transform = Node(
        package='rtk_localization',
        executable='transform',
        name='transform',
        output='screen',
    )
    return [
        antena_arg,
        antenna_split,
        cord_convert,
        antenna_fuse,
        gps_to_enu,
        odometry,
        transform,
    ]

def generate_launch_description(*args, **kwargs):
    return LaunchDescription(get_ros2_nodes())
