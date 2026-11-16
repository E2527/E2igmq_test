"""
フィールドのファイルから地図を作って /map に出すノード.

    ros2 launch field_map field_map.launch.py              # fields/example.yaml、rviz2 も開く
    ros2 run field_map field_map_node --ros-args -p field:=my_field

出すもの:
    /map          (nav_msgs/OccupancyGrid)… Nav2(AMCL・経路計画)と rviz2 が使う地図。
                                             あとから起動したノードにも届くように、最後の地図を残しておく
    /diagnostics  … 地図を作れたか(ロボットコンソールの「状態」に出る)

ファイルを書き換えて保存すると、1 秒以内に地図を作り直して出し直す(起動し直さなくてよい)。
書き方をまちがえたときは、前の地図のまま、/diagnostics とログに理由を出す。
"""

import os

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from . import field as fm

MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FieldMapNode(Node):

    def __init__(self):
        super().__init__('field_map')
        p = self.declare_parameter
        self.path = fm.resolve_field_path(p('field', 'example').value)
        self.frame_id = p('frame_id', 'map').value
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', MAP_QOS)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.stamp = None           # 最後に読んだときのファイルの更新時刻
        self.level, self.message, self.summary = DiagnosticStatus.WARN, 'まだ読んでいません', {}
        self.reload()
        self.create_timer(p('watch_period_s', 1.0).value, self.reload)
        self.create_timer(1.0, self.publish_diagnostics)

    def reload(self):
        """ファイルが変わっていたら(または最初は)、地図を作り直して出す."""
        try:
            stamp = os.stat(self.path).st_mtime_ns
        except OSError:
            self.fail(f'{self.path} がありません')
            return
        if stamp == self.stamp:
            return
        self.stamp = stamp
        try:
            field = fm.load_field(self.path)
            cells = fm.rasterize(field)
        except fm.FieldError as e:
            self.fail(f'書き方がおかしいので、前の地図のままです: {e}')
            return
        self.map_pub.publish(self.to_message(field, cells))
        blocked = sum(1 for v in cells if v == fm.OCCUPIED)
        self.level, self.message = DiagnosticStatus.OK, f'{field.name} の地図を出しています'
        self.summary = {'ファイル': self.path, '広さ [m]': f'{field.x_max - field.x_min:g} × '
                        f'{field.y_max - field.y_min:g}', 'マス': f'{field.width} × {field.height}',
                        '図形の数': str(len(field.shapes)),
                        'ふさがっているマス': f'{100.0 * blocked / len(cells):.1f} %'}
        self.get_logger().info(f'地図を作りました: {field.name}({field.width} × {field.height} マス、'
                               f'図形 {len(field.shapes)} 個)')
        self.publish_diagnostics()

    def fail(self, message):
        if self.message != message:
            self.get_logger().error(message)
        self.level, self.message = DiagnosticStatus.ERROR, message
        self.publish_diagnostics()

    def to_message(self, field, cells):
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = float(field.resolution)
        msg.info.width, msg.info.height = field.width, field.height
        msg.info.origin.position.x = float(field.x_min)
        msg.info.origin.position.y = float(field.y_min)
        msg.info.origin.orientation.w = 1.0
        msg.data = cells
        return msg

    def publish_diagnostics(self):
        values = [KeyValue(key=k, value=v) for k, v in self.summary.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [DiagnosticStatus(name='フィールドの地図', hardware_id='field_map',
                                       level=self.level, message=self.message, values=values)]
        self.diag_pub.publish(msg)


def main():
    rclpy.init()
    node = FieldMapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
