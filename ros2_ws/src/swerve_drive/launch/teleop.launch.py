"""
PC 側: ロボットコンソール(操縦・速さ・状態・停止)と、ゲームパッドを起動する.

    ros2 launch swerve_drive teleop.launch.py              # コンソール + ゲームパッド(./console.sh)
    ros2 launch swerve_drive teleop.launch.py joy:=false   # コンソールだけ(WASD で操縦)
    ros2 launch swerve_drive teleop.launch.py rviz:=true   # rviz2 も開く

    simulator … true: ロボットコンソールの代わりに、シミュレーター(シミュレーションを操る機能つき)を開く。
                swerve.launch.py / omni.launch.py が sim:=true のときに使う

ロボット(Pi)では swerve.launch.py か omni.launch.py を起動しておく。PC と Pi が同じ Wi-Fi にいて
ROS_DOMAIN_ID が同じなら、/cmd_vel は Wi-Fi で Pi に届き、/odom や状態は PC に届く。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('swerve_drive')
    joy_config = os.path.join(share, 'config', 'joy.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('console', default_value='true'),
        DeclareLaunchArgument('joy', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('simulator', default_value='false'),

        # ロボットコンソール(WASD の操縦、速さ、軌跡、状態、停止ボタン)。実機用
        Node(package='swerve_drive', executable='robot_console', name='robot_console',
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('console'), "' == 'true' and '",
                 LaunchConfiguration('simulator'), "' != 'true'"]))),
        # シミュレーター(ロボットコンソール + シミュレーションを操る機能)。sim のとき
        Node(package='swerve_drive', executable='simulator', name='simulator',
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('console'), "' == 'true' and '",
                 LaunchConfiguration('simulator'), "' == 'true'"]))),

        # ゲームパッド → /joy → /cmd_vel(LB を押している間だけ。config/joy.yaml)
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[joy_config], condition=IfCondition(LaunchConfiguration('joy'))),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
             parameters=[joy_config], condition=IfCondition(LaunchConfiguration('joy'))),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(share, 'config', 'swerve.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
