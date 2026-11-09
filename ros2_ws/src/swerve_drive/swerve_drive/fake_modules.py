"""
実機の代わりに、スワーブのモジュール(STM32 + モーター)のふりをするノード(動作確認用).

stm_bridge と STM32 の代わりに、トピックで直接やり取りする:
    stm<ID>/command [向き rad, タイヤの速さ rad/s] を受け取り、
    stm<ID>/state   [今の向き rad, 今のタイヤの速さ rad/s] を返す

    ・向きは最大 steer_speed [rad/s] で、近いほうの回り方で目標に近づく
    ・タイヤの速さは wheel_time_constant [s] でなめらかに目標に近づく
    ・指令が timeout_s 以上届かなければタイヤを止める(STM32 と同じ)

    ros2 run swerve_drive fake_modules
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .kinematics import wrap_angle


class FakeModules(Node):

    def __init__(self):
        super().__init__('fake_modules')
        self.ids = list(self.declare_parameter('device_ids', [1, 2, 3, 4]).value)
        self.steer_speed = self.declare_parameter('steer_speed', 6.0).value
        self.wheel_tc = self.declare_parameter('wheel_time_constant', 0.1).value
        self.timeout_s = self.declare_parameter('timeout_s', 0.1).value
        rate_hz = self.declare_parameter('rate_hz', 100.0).value

        n = len(self.ids)
        self.targets = [(0.0, 0.0, 0.0)] * n     # (受け取った時刻, 向き, 速さ)
        self.angles = [0.0] * n
        self.speeds = [0.0] * n
        self.pubs = []
        for i, dev_id in enumerate(self.ids):
            self.pubs.append(self.create_publisher(Float32MultiArray, f'stm{dev_id}/state', 10))
            self.create_subscription(
                Float32MultiArray, f'stm{dev_id}/command',
                lambda msg, i=i: self.on_command(i, msg), 10)
        self.dt = 1.0 / rate_hz
        self.create_timer(self.dt, self.step)
        self.get_logger().info(f'ID {self.ids} のモジュールのふりをしています')

    def on_command(self, index, msg):
        if len(msg.data) >= 2:
            self.targets[index] = (time.monotonic(), float(msg.data[0]), float(msg.data[1]))

    def step(self):
        now = time.monotonic()
        gain = min(1.0, self.dt / self.wheel_tc)
        max_turn = self.steer_speed * self.dt
        for i, (received, angle, speed) in enumerate(self.targets):
            if now - received > self.timeout_s:
                speed = 0.0     # 指令が途切れたら止まる(向きはそのまま)
                angle = self.angles[i]
            turn = wrap_angle(angle - self.angles[i])
            self.angles[i] = wrap_angle(self.angles[i] + max(-max_turn, min(max_turn, turn)))
            self.speeds[i] += gain * (speed - self.speeds[i])
            self.pubs[i].publish(Float32MultiArray(data=[self.angles[i], self.speeds[i]]))


def main():
    rclpy.init()
    node = FakeModules()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
