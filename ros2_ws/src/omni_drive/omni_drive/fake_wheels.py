"""
実機の代わりに、STM32(モーター 2 個の速度制御)のふりをするノード(動作確認用).

stm_bridge と STM32 の代わりに、トピックで直接やり取りする:
    stm<ID>/command [モーター1 の目標速度, モーター2 の目標速度] を受け取り、
    stm<ID>/state   [モーター1 の実際の速度, モーター2 の実際の速度] を返す

    ・速さは time_constant [s] でなめらかに目標に近づく(モーター + PID の応答の代わり)
    ・指令が timeout_s 以上届かなければ止まる(STM32 と同じ)

    ros2 run omni_drive fake_wheels
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class FakeWheels(Node):

    def __init__(self):
        super().__init__('fake_wheels')
        self.boards = list(self.declare_parameter('boards', [1, 2]).value)
        self.time_constant = self.declare_parameter('time_constant', 0.1).value
        self.timeout_s = self.declare_parameter('timeout_s', 0.1).value
        rate_hz = self.declare_parameter('rate_hz', 100.0).value

        self.targets = {b: (0.0, [0.0, 0.0]) for b in self.boards}   # (受け取った時刻, 目標)
        self.speeds = {b: [0.0, 0.0] for b in self.boards}
        self.pubs = {}
        for board in self.boards:
            self.pubs[board] = self.create_publisher(Float32MultiArray, f'stm{board}/state', 10)
            self.create_subscription(
                Float32MultiArray, f'stm{board}/command',
                lambda msg, b=board: self.on_command(b, msg), 10)
        self.dt = 1.0 / rate_hz
        self.create_timer(self.dt, self.step)
        self.get_logger().info(f'ID {self.boards} の STM32(モーター 2 個)のふりをしています')

    def on_command(self, board, msg):
        if len(msg.data) >= 2:
            self.targets[board] = (time.monotonic(), [float(msg.data[0]), float(msg.data[1])])

    def step(self):
        now = time.monotonic()
        gain = min(1.0, self.dt / self.time_constant)
        for board, (received, target) in self.targets.items():
            if now - received > self.timeout_s:
                target = [0.0, 0.0]     # 指令が途切れたら止まる
            speed = self.speeds[board]
            for k in range(2):
                speed[k] += gain * (target[k] - speed[k])
            self.pubs[board].publish(Float32MultiArray(data=list(speed)))


def main():
    rclpy.init()
    node = FakeWheels()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
