"""
シミュレーションの「世界」: フィールドの地図と、シミュレーション用の LiDAR.

swerve_drive / omni_drive の起動ファイルが、sim:=true のときに読み込む。

    field     … fields/ の中のフィールドの名前(example)
    lidar     … true: シミュレーション用の LiDAR(fake_lidar)で /scan を出す
    start     … フィールドの start_poses の名前(red など)。空ならフィールドの (0, 0) から
    publish_map_to_odom … true: TF の map → odom を出す。AMCL を使うときは false(AMCL が出す)

sim_world_node がロボットの本当の位置を持ち、TF の map → odom を出す(シミュレーターで置き直せる)。
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
        DeclareLaunchArgument('start', default_value=''),
        DeclareLaunchArgument('publish_map_to_odom', default_value='true'),
        Node(package='field_map', executable='field_map_node', name='field_map',
             parameters=[{'field': LaunchConfiguration('field')}], output='screen'),
        Node(package='field_map', executable='fake_lidar', name='fake_lidar', output='screen',
             condition=IfCondition(LaunchConfiguration('lidar'))),
        Node(package='field_map', executable='sim_world_node', name='sim_world', output='screen',
             parameters=[{'field': LaunchConfiguration('field'),
                          'start': LaunchConfiguration('start'),
                          'publish_map_to_odom': LaunchConfiguration('publish_map_to_odom')}]),
    ])
