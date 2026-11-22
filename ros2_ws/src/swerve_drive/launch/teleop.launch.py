"""
PC 側: ロボットコンソール(操縦・速さ・状態・停止)と、ゲームパッドを起動する.

    ros2 launch swerve_drive teleop.launch.py              # コンソール + ゲームパッド(./console.sh)
    ros2 launch swerve_drive teleop.launch.py joy:=false   # コンソールだけ(WASD で操縦)
    ros2 launch swerve_drive teleop.launch.py rviz:=true   # rviz2 も開く
    ros2 launch swerve_drive teleop.launch.py nav:=true field:=example   # 自動操縦(Nav2)も PC で動かす

    simulator … true: ロボットコンソールの代わりに、シミュレーター(シミュレーションを操る機能つき)を開く。
                swerve.launch.py / omni.launch.py が sim:=true のときに使う

    nav       … true: 自動操縦(robot_nav の Nav2)を、このコンピューターで起動する(実機用。
                AMCL も起動する)。field … そのときのフィールドの名前
    amcl      … true: 自分の位置を合わせる AMCL だけを、このコンピューターで起動する(実機用。
                LiDAR の /scan で地図とのずれを直す。地図のノードも起動する)

ロボット(Pi)では swerve.launch.py か omni.launch.py を起動しておく。PC と Pi が同じ Wi-Fi にいて
ROS_DOMAIN_ID が同じなら、指令は Wi-Fi で Pi に届き、/odom や状態は PC に届く。
手で操縦する指令(コンソール・ゲームパッド)は cmd_vel_teleop に出す。Pi の cmd_vel_mux が、
自動操縦の指令(cmd_vel_nav)より優先して /cmd_vel にまとめる。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('swerve_drive')
    joy_config = os.path.join(share, 'config', 'joy.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('console', default_value='true'),
        DeclareLaunchArgument('joy', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('simulator', default_value='false'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('amcl', default_value='false'),
        DeclareLaunchArgument('field', default_value='example'),

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

        # ゲームパッド → /joy → cmd_vel_teleop(LB を押している間だけ。config/joy.yaml)
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[joy_config], condition=IfCondition(LaunchConfiguration('joy'))),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
             parameters=[joy_config], remappings=[('cmd_vel', 'cmd_vel_teleop')],
             condition=IfCondition(LaunchConfiguration('joy'))),

        # 実機で、PC で地図のノードと AMCL を動かすとき(nav でも amcl でも)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('robot_nav'), 'launch', 'localization.launch.py'])),
            launch_arguments={'field': LaunchConfiguration('field'), 'map': 'true',
                              'amcl': 'true'}.items(),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('nav'), "' == 'true' or '",
                LaunchConfiguration('amcl'), "' == 'true'"]))),
        # 自動操縦(実機で、PC で Nav2 を動かすとき)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('robot_nav'), 'launch', 'nav.launch.py'])),
            launch_arguments={'field': LaunchConfiguration('field'), 'map': 'false',
                              'amcl': 'false'}.items(),
            condition=IfCondition(LaunchConfiguration('nav'))),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(share, 'config', 'swerve.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
