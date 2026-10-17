import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('stm_bridge'), 'config', 'bridge.yaml')
    return LaunchDescription([
        Node(
            package='stm_bridge',
            executable='bridge',
            name='stm_bridge',
            parameters=[config],
            output='screen',
        ),
    ])
