"""
スワーブの足回りを起動する.

    # 実機なしで試す(モジュールのふり + ロボットコンソール。WASD で操縦)
    ros2 launch swerve_drive swerve.launch.py sim:=true console:=true
    # STM32 のファームウェア(ゲイン・ギア比)まで含めて試す
    ros2 launch swerve_drive swerve.launch.py sim:=true firmware:=true console:=true
    # ゲームパッドでも操縦する / rviz2 も開く
    ros2 launch swerve_drive swerve.launch.py sim:=true console:=true joy:=true rviz:=true

    # 実機(stm_bridge で STM32 × 4 とつなぐ)。PC から Wi-Fi で操縦するときは joy なしで起動し、
    # PC で teleop.launch.py を起動する
    ros2 launch swerve_drive swerve.launch.py can_interface:=can0

引数:
    sim           … true: fake_modules で実機のふりをする / false: stm_bridge を起動して実機とつなぐ
    firmware      … true(sim のとき): fake_modules の代わりに、STM32 のファームウェアをそのまま動かす
                    firmware_modules を使う(app_config.h のゲイン・ギア比が動きに出る)
    console       … true: ロボットコンソール(robot_console)を開く。WASD の操縦、速さ、軌跡、
                    状態(/diagnostics)、停止ボタンが 1 つのウィンドウにまとまっている(PC 用)
    joy           … true: このコンピューターにつないだゲームパッドで /cmd_vel を出す
    rviz          … true: rviz2 でオドメトリと各モジュールの向きを表示する(PC 用)
    can_interface … stm_bridge の通信の線(can0 / serial:/dev/ttyACM0)。
                    指定しなければ環境変数 STM_INTERFACE、それもなければ bridge.yaml の値

/cmd_vel を出すものは何でもよい(ゲームパッド、キーボード、Nav2)。自動操縦にするときも、
このファイルはそのままで、Nav2 が /cmd_vel を出すようにすればよい。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _modules(context):
    """
    STM32 の側: sim なら fake_modules か firmware_modules、実機なら stm_bridge.

    どちらも ID は swerve.yaml の device_ids を使う(1 か所だけ書けばよい)。
    """
    share = get_package_share_directory('swerve_drive')
    config = os.path.join(share, 'config', 'swerve.yaml')
    with open(config, encoding='utf-8') as f:
        ids = yaml.safe_load(f)['swerve_drive']['ros__parameters']['device_ids']
    if IfCondition(LaunchConfiguration('sim')).evaluate(context):
        use_firmware = IfCondition(LaunchConfiguration('firmware')).evaluate(context)
        name = 'firmware_modules' if use_firmware else 'fake_modules'
        return [Node(package='swerve_drive', executable=name, name=name,
                     parameters=[config, {'device_ids': ids}], output='screen')]
    bridge_config = os.path.join(get_package_share_directory('stm_bridge'), 'config',
                                 'bridge.yaml')
    overrides = {'device_ids': ids}
    interface = LaunchConfiguration('can_interface').perform(context)
    if interface:
        overrides['can_interface'] = interface
    return [Node(package='stm_bridge', executable='bridge', name='stm_bridge',
                 parameters=[bridge_config, overrides], output='screen')]


def generate_launch_description():
    share = get_package_share_directory('swerve_drive')
    swerve_config = os.path.join(share, 'config', 'swerve.yaml')
    joy = LaunchConfiguration('joy')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('firmware', default_value='false'),
        DeclareLaunchArgument('console', default_value='false'),
        DeclareLaunchArgument('joy', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'can_interface', default_value=EnvironmentVariable('STM_INTERFACE', default_value='')),

        # 足回りの計算(/cmd_vel → 各モジュール、各モジュール → /odom)
        Node(package='swerve_drive', executable='swerve_node', name='swerve_drive',
             parameters=[swerve_config], output='screen'),

        # 実機のふり / 実機とつなぐ
        OpaqueFunction(function=_modules),

        # 操縦と表示(PC で)。PC から Wi-Fi で操縦するときは、PC で teleop.launch.py を起動する
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'teleop.launch.py')),
            launch_arguments={'console': LaunchConfiguration('console'), 'joy': joy,
                              'rviz': 'false'}.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(share, 'config', 'swerve.rviz')],
             condition=IfCondition(rviz)),
    ])
