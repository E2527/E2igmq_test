"""
Nav2(自動操縦)を起動する.

    # シミュレーションで(./sim.sh nav が、足回りの起動ファイル経由でこれを読み込む)
    ./sim.sh nav
    # 実機で(PC で。ロボットコンソールと一緒に)
    ./console.sh nav:=true field:=example

引数:
    field       … フィールドの名前(field_map の fields/)。map:=true のときに使う
    map         … true: フィールドの地図のノード(field_map_node)も起動する。
                  シミュレーションでは sim_world.launch.py が起動するので false
    amcl        … true: AMCL で自分の位置を合わせる(LiDAR と地図を重ねる。実機では必ず true)。
                  false: TF の map → odom はほかのもの(シミュレーションの sim_world_node)にまかせる
    params_file … Nav2 の設定(config/nav2_params.yaml)

出すもの: /cmd_vel_nav(足回りの cmd_vel_mux が /cmd_vel にまとめる)、/plan(探した道)
受け取るもの: ゴール(navigate_to_pose。ロボットコンソールの「自動操縦」タブで地図をクリック)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 自動操縦の部品(この順番で起動する)
NAV_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']


def generate_launch_description():
    share = get_package_share_directory('robot_nav')
    params = LaunchConfiguration('params_file')
    # Nav2 は cmd_vel に出すので、cmd_vel_nav に名前を変える(手で操縦する方が優先されるように)
    to_nav = [('cmd_vel', 'cmd_vel_nav')]
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

        # 自分の位置を合わせる(AMCL)。地図が届いてから動き出す
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params], condition=IfCondition(LaunchConfiguration('amcl'))),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True, 'node_names': ['amcl']}],
             condition=IfCondition(LaunchConfiguration('amcl'))),

        # 道を探して、道に沿って走る
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[params], remappings=to_nav),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[params], remappings=to_nav),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[params]),
        # 上の部品を順番に起動する(Nav2 の部品は「設定 → 起動」の 2 段階で動き出す)
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True, 'node_names': NAV_NODES}]),
    ])
