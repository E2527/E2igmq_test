"""
ロボットの状態を知らせるための共通の部品(足回りのノードとダッシュボードで使う).

    /diagnostics(diagnostic_msgs/DiagnosticArray)… ROS 2 の共通の決まり。各ノードが自分の状態を
        「OK / 注意(WARN)/ エラー(ERROR)」と中身で出す。rqt_robot_monitor や robot_dashboard で見られる
    /drive_stop(std_msgs/Bool)… 停止ボタン。true の間、足回りは /cmd_vel を無視して止まる
        最後の値を覚えておく設定(transient local)なので、あとから起動したノードにも届く
"""

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR

STOP_TOPIC = 'drive_stop'
# 停止ボタンの状態は、あとから起動したノードにも最後の値が届くようにする
STOP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def make_status(name: str, level, message: str, hardware_id: str = '', **values):
    """診断の 1 項目を作る。values は「名前=値」の表として一緒に表示される."""
    return DiagnosticStatus(
        name=name, level=level, message=message, hardware_id=hardware_id,
        values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
