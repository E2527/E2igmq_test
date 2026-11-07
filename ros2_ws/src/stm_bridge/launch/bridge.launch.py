"""
stm_bridge を config/bridge.yaml の設定で起動する.

    ros2 launch stm_bridge bridge.launch.py                                   # bridge.yaml のまま
    ros2 launch stm_bridge bridge.launch.py can_interface:=serial:/dev/ttyACM0 # 通信の線だけ変える

can_interface を指定しないときは、環境変数 STM_INTERFACE、それもなければ bridge.yaml の値を使う
(Pi の compose.yaml の stm-bridge は STM_INTERFACE で切り替える)。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _bridge_node(context):
    config = os.path.join(get_package_share_directory('stm_bridge'), 'config', 'bridge.yaml')
    parameters = [config]
    interface = LaunchConfiguration('can_interface').perform(context)
    if interface:
        parameters.append({'can_interface': interface})
    return [Node(
        package='stm_bridge',
        executable='bridge',
        name='stm_bridge',
        parameters=parameters,
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface',
            default_value=EnvironmentVariable('STM_INTERFACE', default_value=''),
            description='can0 / serial:/dev/ttyACM0 / sim(空なら bridge.yaml の値)'),
        OpaqueFunction(function=_bridge_node),
    ])
