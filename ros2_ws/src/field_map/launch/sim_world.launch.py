"""
シミュレーションの「世界」: フィールドの地図と、シミュレーション用の LiDAR.

swerve_drive / omni_drive の起動ファイルが、sim:=true のときに読み込む。

    field     … fields/ の中のフィールドの名前(example)
    lidar     … true: シミュレーション用の LiDAR(fake_lidar)で /scan を出す

ロボットはフィールドの (0, 0) から走り出す。自分の位置を合わせる部品(AMCL)がまだないので、
地図(map)とオドメトリ(odom)の座標を同じにする TF(map → odom)も出す。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('field', default_value='example'),
        DeclareLaunchArgument('lidar', default_value='true'),
        Node(package='field_map', executable='field_map_node', name='field_map',
             parameters=[{'field': LaunchConfiguration('field')}], output='screen'),
        Node(package='field_map', executable='fake_lidar', name='fake_lidar', output='screen',
             condition=IfCondition(LaunchConfiguration('lidar'))),
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'], output='log'),
    ])
