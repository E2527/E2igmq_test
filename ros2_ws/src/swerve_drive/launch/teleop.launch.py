"""
PC 側: ゲームパッドで /cmd_vel を出し、rviz2 でロボットの様子を見る.

    ros2 launch swerve_drive teleop.launch.py              # ゲームパッド + rviz2
    ros2 launch swerve_drive teleop.launch.py rviz:=false  # ゲームパッドだけ

ロボット(Pi)では swerve.launch.py を起動しておく。PC と Pi が同じ Wi-Fi にいて
ROS_DOMAIN_ID が同じなら、/cmd_vel は Wi-Fi で Pi に届き、/odom や矢印は PC に届く。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('swerve_drive')
    joy_config = os.path.join(share, 'config', 'joy.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),

        # ゲームパッド → /joy → /cmd_vel(LB を押している間だけ。config/joy.yaml)
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[joy_config]),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
             parameters=[joy_config]),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(share, 'config', 'swerve.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
