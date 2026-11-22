"""
止まっているときも、定期的に AMCL に位置を直させるノード.

AMCL は、ロボットが動いたとき(update_min_d / update_min_a)しか、LiDAR と地図を見比べない。
止まったまま位置がずれていると(置き直した直後、ぶつかって押されたときなど)、ずっと直らない。
このノードは、止まっている間 period_s 秒ごとに、AMCL の request_nomotion_update を呼んで、
LiDAR と地図を見比べ直させる(動いている間は AMCL が自分で直すので、呼ばない)。

    受け取るもの: /odom(止まっているか)
    呼ぶもの: request_nomotion_update(std_srvs/Empty。AMCL)
"""

import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Empty

from .relocalize_timing import should_request


class Relocalize(Node):

    def __init__(self):
        super().__init__('relocalize')
        p = self.declare_parameter
        self.period_s = p('period_s', 3.0).value            # 止まっている間、何秒ごとに直すか(0 でしない)
        self.still_speed = p('still_speed', 0.02).value     # これより遅ければ、止まっているとみなす
        self.speed = None
        self.speed_time = None
        self.last_request = None
        self.client = self.create_client(Empty, 'request_nomotion_update')
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(0.5, self.tick)

    def on_odom(self, msg):
        t = msg.twist.twist
        self.speed = max(math.hypot(t.linear.x, t.linear.y), abs(t.angular.z))
        self.speed_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        speed = self.speed if self.speed_time and now - self.speed_time < 1.0 else None
        if not self.client.service_is_ready():
            return
        if should_request(now, self.last_request, self.period_s, speed, self.still_speed):
            self.last_request = now
            self.client.call_async(Empty.Request())


def main():
    rclpy.init()
    node = Relocalize()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
