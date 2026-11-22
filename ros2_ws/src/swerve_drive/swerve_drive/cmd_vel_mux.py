"""
速さの指令をまとめて /cmd_vel に出すノード(手で操縦する方が、自動操縦より優先).

    cmd_vel_teleop(geometry_msgs/Twist)… ロボットコンソール・ゲームパッドの指令(優先度 高)
    cmd_vel_nav   (geometry_msgs/Twist)… Nav2 の指令(優先度 低)
        ↓ 優先度のいちばん高いものだけ
    cmd_vel       (geometry_msgs/Twist)… 足回り(swerve_node / omni_node)へ
    cmd_vel_mux/active(std_msgs/String)… 今どれを通しているか(teleop / nav / 空。コンソールが表示する)

足回りの起動ファイル(swerve.launch.py / omni.launch.py)が、足回りと一緒に起動する。
選び方の計算は cmd_mux.py。
"""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .cmd_mux import CmdMux


class CmdVelMux(Node):

    def __init__(self):
        super().__init__('cmd_vel_mux')
        p = self.declare_parameter
        # 手で操縦する指令がこれだけ途切れたら、自動操縦にもどす [s]
        teleop_timeout = p('teleop_timeout_s', 0.5).value
        nav_timeout = p('nav_timeout_s', 0.5).value
        self.mux = CmdMux([('teleop', 100, teleop_timeout), ('nav', 10, nav_timeout)])
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.active_pub = self.create_publisher(String, 'cmd_vel_mux/active', 10)
        for name in ('teleop', 'nav'):
            self.create_subscription(Twist, f'cmd_vel_{name}',
                                     lambda msg, n=name: self.on_input(n, msg), 10)
        self.create_timer(0.2, self.publish_active)

    def on_input(self, name, msg):
        if self.mux.on_input(name, time.monotonic()):
            self.pub.publish(msg)

    def publish_active(self):
        self.active_pub.publish(String(data=self.mux.active(time.monotonic()) or ''))


def main():
    rclpy.init()
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
