"""
STM32 のファームウェア(モーター 2 個の速度制御、APP なしの通常版)をそのまま動かして、STM32 のふりをするノード.

fake_wheels(簡単な真似)の代わりに使う。ファームウェアの速度の測り方と PID、設定(app_config.h)を
PC でビルドして動かし、モーターは計算の模型で真似する(firmware/sim/firmware_sim.c)。

    stm<ID>/command [モーター1 の目標速度, モーター2 の目標速度] を受け取り、
    stm<ID>/state   [モーター1 の速度, モーター2 の速度](ファームウェアが Pi に返すのと同じ値)を返す

    ros2 launch omni_drive omni.launch.py sim:=true firmware:=true rviz:=true keyboard:=true
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from swerve_drive import firmware_sim


class FirmwareWheels(Node):

    def __init__(self):
        super().__init__('firmware_wheels')
        p = self.declare_parameter
        self.boards = list(p('boards', [1, 2]).value)
        firmware_dir = p('firmware_dir', firmware_sim.DEFAULT_FIRMWARE_DIR).value
        no_load_speed = p('motor_no_load_speed', 0.0).value      # 0 ならファームウェアの設定に合わせる
        time_constant = p('motor_time_constant', 0.0).value
        self.timeout_s = p('timeout_s', 0.1).value
        rate_hz = p('rate_hz', 100.0).value

        lib = firmware_sim.build(firmware_dir)
        self.sims = {b: firmware_sim.BoardSim(lib, no_load_speed, time_constant)
                     for b in self.boards}
        self.targets = {b: (0.0, [0.0, 0.0]) for b in self.boards}
        self.pubs = {}
        for board in self.boards:
            self.pubs[board] = self.create_publisher(Float32MultiArray, f'stm{board}/state', 10)
            self.create_subscription(
                Float32MultiArray, f'stm{board}/command',
                lambda msg, b=board: self.on_command(b, msg), 10)
        self.period = 1.0 / rate_hz
        self.create_timer(self.period, self.step)
        self.get_logger().info(
            f'ID {self.boards} の STM32 を、{firmware_dir} のファームウェアで動かしています')

    def on_command(self, board, msg):
        if len(msg.data) >= 2:
            self.targets[board] = (time.monotonic(), [float(msg.data[0]), float(msg.data[1])])

    def step(self):
        now = time.monotonic()
        for board, (received, target) in self.targets.items():
            running = now - received < self.timeout_s
            speeds = self.sims[board].run(target[0], target[1], running, self.period)
            self.pubs[board].publish(Float32MultiArray(data=speeds))

    def destroy_node(self):
        for sim in self.sims.values():
            sim.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = FirmwareWheels()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
