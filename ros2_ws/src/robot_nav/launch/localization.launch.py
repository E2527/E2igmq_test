"""
自分の位置を合わせる(AMCL)を起動する。自動操縦(nav.launch.py)がなくても使える.

    ./sim.sh            # シミュレーターでは、ふつうに起動する(./sim.sh noamcl で使わない)
    ./console.sh amcl:=true field:=example     # 実機で(PC で)

LiDAR の点(/scan)と、LiDAR に映る物だけの地図(/lidar_map)を重ねて、ロボットが地図のどこにいるかを
直し、TF の map → odom を出す。タイヤが滑ってオドメトリがずれても、地図とのずれが直る。

    ロボットが動いたとき … AMCL が自分で直す(update_min_d / update_min_a だけ動くたび)
    止まっているとき     … relocalize が period_s 秒ごとに、AMCL に直すように頼む

引数:
    map         … true: フィールドの地図のノード(field_map_node)も起動する
    amcl        … false: 何も起動しない(起動ファイルから、条件つきで読み込むため)
    field       … フィールドの名前(map:=true のとき)
    params_file … 設定(config/nav2_params.yaml の amcl と relocalize)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_nav')
    params = LaunchConfiguration('params_file')
    amcl = IfCondition(LaunchConfiguration('amcl'))
    return LaunchDescription([
        DeclareLaunchArgument('field', default_value='example'),
        DeclareLaunchArgument('map', default_value='true'),
        DeclareLaunchArgument('amcl', default_value='true'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'nav2_params.yaml')),

        # フィールドの地図(/map と /lidar_map)
        Node(package='field_map', executable='field_map_node', name='field_map', output='screen',
             parameters=[{'field': LaunchConfiguration('field')}],
             condition=IfCondition(LaunchConfiguration('map'))),

        # AMCL。地図が届いてから動き出す
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params], condition=amcl),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True, 'node_names': ['amcl']}], condition=amcl),
        # 止まっているときも、定期的に直す
        Node(package='robot_nav', executable='relocalize', name='relocalize', output='screen',
             parameters=[params], condition=amcl),
    ])
