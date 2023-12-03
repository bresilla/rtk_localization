from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def get_ros2_nodes():
    single_gps = Node(
        package='rtk_localization',
        executable='single_gps',
        name='single_gps',
        output='screen',
    )
    cord_convert = Node(
        package='rtk_localization',
        executable='cord_convert',
        name='cord_convert',
        output='screen',
    )
    gps_fuse = Node(
        package='rtk_localization',
        executable='gps_fuse',
        name='gps_fuse',
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
        cord_convert,
        single_gps,
        gps_fuse,
        gps_to_enu,
        odometry,
        transform,
    ]

def generate_launch_description(*args, **kwargs):
    arg1 = DeclareLaunchArgument('delta', default_value='0.1', description='Threshold for delta')
    arg2 = DeclareLaunchArgument('double_antena', default_value='Trfue', description='if true, use double antena')
    return LaunchDescription(
        [
            arg1,
            arg2,
        ] 
    + get_ros2_nodes(
    )
)
