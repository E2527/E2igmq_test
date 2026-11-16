"""
フィールドの地図を出す(rviz2 で見る).

    ros2 launch field_map field_map.launch.py                      # fields/example.yaml、rviz2 も開く
    ros2 launch field_map field_map.launch.py field:=my_field      # fields/my_field.yaml
    ros2 launch field_map field_map.launch.py rviz:=false          # 地図だけ(Pi など)

    field … fields/ の中のファイルの名前(.yaml なし)か、ファイルのパス
    rviz  … true なら rviz2 で地図を表示する
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('field_map')
    return LaunchDescription([
        DeclareLaunchArgument('field', default_value='example'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='field_map', executable='field_map_node', name='field_map',
             parameters=[{'field': LaunchConfiguration('field')}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'config', 'field_map.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
