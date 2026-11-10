"""
オムニホイールの足回りを起動する.

    # 実機なしで試す(STM32 のふり + rviz2 で確認 + WASD かゲームパッド)
    ros2 launch omni_drive omni.launch.py sim:=true rviz:=true keyboard:=true

    # 実機(stm_bridge で STM32 とつなぐ。STM32 はモーター 2 個の速度制御の通常版)
    ros2 launch omni_drive omni.launch.py can_interface:=serial:/dev/ttyACM0

引数(swerve_drive の swerve.launch.py と同じ):
    sim           … true: fake_wheels で実機のふりをする / false: stm_bridge を起動して実機とつなぐ
    joy           … true: このコンピューターにつないだゲームパッドで /cmd_vel を出す
    keyboard      … true: WASD で /cmd_vel を出す(swerve_drive の keyboard_teleop)
    rviz          … true: rviz2 でオドメトリと各ホイールの矢印を表示する(PC 用)
    can_interface … stm_bridge の通信の線(can0 / serial:/dev/ttyACM0)。
                    指定しなければ環境変数 STM_INTERFACE、それもなければ bridge.yaml の値

PC から Wi-Fi で操縦するときは、PC で swerve_drive の teleop.launch.py を起動する(/cmd_vel は同じ)。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _bridge(context):
    """実機のときの stm_bridge。device_ids は omni.yaml に出てくる基板の ID にする."""
    share = get_package_share_directory('omni_drive')
    with open(os.path.join(share, 'config', 'omni.yaml'), encoding='utf-8') as f:
        ids = yaml.safe_load(f)['omni_drive']['ros__parameters']['device_ids']
    bridge_config = os.path.join(get_package_share_directory('stm_bridge'), 'config',
                                 'bridge.yaml')
    overrides = {'device_ids': sorted(set(ids))}
    interface = LaunchConfiguration('can_interface').perform(context)
    if interface:
        overrides['can_interface'] = interface
    return [Node(
        package='stm_bridge', executable='bridge', name='stm_bridge',
        parameters=[bridge_config, overrides], output='screen',
        condition=UnlessCondition(LaunchConfiguration('sim')))]


def generate_launch_description():
    share = get_package_share_directory('omni_drive')
    config = os.path.join(share, 'config', 'omni.yaml')
    teleop = os.path.join(get_package_share_directory('swerve_drive'), 'launch',
                          'teleop.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('joy', default_value='false'),
        DeclareLaunchArgument('keyboard', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'can_interface', default_value=EnvironmentVariable('STM_INTERFACE', default_value='')),

        # 足回りの計算(/cmd_vel → 各ホイール、各ホイール → /odom)
        Node(package='omni_drive', executable='omni_node', name='omni_drive',
             parameters=[config], output='screen'),

        # 実機のふり / 実機とつなぐ
        Node(package='omni_drive', executable='fake_wheels', name='fake_wheels',
             parameters=[config], output='screen',
             condition=IfCondition(LaunchConfiguration('sim'))),
        OpaqueFunction(function=_bridge),

        # 操縦(swerve_drive と共通)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop),
            launch_arguments={'rviz': 'false'}.items(),
            condition=IfCondition(LaunchConfiguration('joy'))),
        Node(package='swerve_drive', executable='keyboard_teleop', name='keyboard_teleop',
             condition=IfCondition(LaunchConfiguration('keyboard'))),

        # 表示(PC で)
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(share, 'config', 'omni.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
