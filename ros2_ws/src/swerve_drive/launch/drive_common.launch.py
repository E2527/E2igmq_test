"""
スワーブとオムニで共通の部分(swerve.launch.py / omni.launch.py が読み込む).

    シミュレーションの世界(sim のとき)… フィールドの地図、シミュレーション用の LiDAR、本当の位置
    cmd_vel_mux … 手で操縦する指令(cmd_vel_teleop)と自動操縦の指令(cmd_vel_nav)をまとめて
                  /cmd_vel に出す。手で操縦する方が優先
    自動操縦(nav のとき)… robot_nav の nav.launch.py(Nav2)
    操縦と表示 … teleop.launch.py(ロボットコンソール / シミュレーター、ゲームパッド)

引数は swerve.launch.py の説明を見る。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _not(name):
    """引数の反対(true なら false、それ以外なら true。起動ファイルの引数の文字のまま)."""
    return PythonExpression(["'false' if '", LaunchConfiguration(name), "' == 'true' else 'true'"])


def _include(package, file, arguments, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), 'launch',
                                                            file])),
        launch_arguments=arguments.items(), condition=condition)


def generate_launch_description():
    sim, nav = LaunchConfiguration('sim'), LaunchConfiguration('nav')
    field, amcl = LaunchConfiguration('field'), LaunchConfiguration('amcl')
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('console', default_value='false'),
        DeclareLaunchArgument('joy', default_value='false'),
        DeclareLaunchArgument('field', default_value='example'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('start', default_value=''),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('amcl', default_value='false'),

        # シミュレーションの世界(sim のとき)。AMCL を使うときは、map → odom を AMCL にまかせる
        _include('field_map', 'sim_world.launch.py',
                 {'field': field, 'lidar': LaunchConfiguration('lidar'),
                  'start': LaunchConfiguration('start'), 'publish_map_to_odom': _not('amcl')},
                 IfCondition(sim)),

        # 手で操縦する指令と、自動操縦の指令をまとめる(手で操縦する方が優先)
        Node(package='swerve_drive', executable='cmd_vel_mux', name='cmd_vel_mux',
             output='screen'),

        # 自動操縦(Nav2)。シミュレーションでは、地図のノードは上の世界が起動している
        _include('robot_nav', 'nav.launch.py',
                 {'field': field, 'map': _not('sim'), 'amcl': amcl}, IfCondition(nav)),

        # 操縦と表示(PC で)。PC から Wi-Fi で操縦するときは、PC で teleop.launch.py を起動する
        _include('swerve_drive', 'teleop.launch.py',
                 {'console': LaunchConfiguration('console'), 'simulator': sim,
                  'joy': LaunchConfiguration('joy'), 'rviz': 'false'}),
    ])
