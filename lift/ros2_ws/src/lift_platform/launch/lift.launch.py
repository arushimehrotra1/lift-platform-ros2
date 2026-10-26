from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        Node(package='lift_platform', executable='serial_bridge', name='lift_serial_bridge',
             output='screen', parameters=[{'port': LaunchConfiguration('port')}]),
    ])
