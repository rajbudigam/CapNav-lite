from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    profile = LaunchConfiguration('profile')
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='profiles/default_adult.json'),
        Node(
            package='capnav_lite_runtime',
            executable='safety_monitor_node',
            name='capnav_safety_monitor',
            parameters=[{'profile_path': profile}],
            output='screen',
        ),
        Node(
            package='capnav_lite_runtime',
            executable='shared_control_node',
            name='capnav_shared_control',
            parameters=[{'profile_path': profile}],
            output='screen',
        ),
    ])
