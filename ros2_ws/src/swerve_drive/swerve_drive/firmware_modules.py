"""
STM32 のファームウェア(APP=swerve)をそのまま動かして、スワーブのモジュールのふりをするノード.

fake_modules(簡単な真似)の代わりに使う。ファームウェアの計算部分と設定(app_config.h の
ゲイン・ギア比・エンコーダ)を PC でビルドして動かし、モーターと差動ギアは計算の模型で真似する
(firmware/sim/firmware_sim.c)。

    stm<ID>/command [向き rad, タイヤの速さ rad/s] を受け取り、
    stm<ID>/state   [向き, タイヤの速さ](ファームウェアが Pi に返すのと同じ値)を返す
    stm<ID>/true_state [本当の向き, 本当のタイヤの速さ](模型の値。ギア比の設定の確認用)

    ros2 launch swerve_drive swerve.launch.py sim:=true firmware:=true rviz:=true keyboard:=true
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from . import firmware_sim


class FirmwareModules(Node):

    def __init__(self):
        super().__init__('firmware_modules')
        p = self.declare_parameter
        self.ids = list(p('device_ids', [1, 2, 3, 4]).value)
        firmware_dir = p('firmware_dir', firmware_sim.DEFAULT_FIRMWARE_DIR).value
        # 模型(0 ならファームウェアの設定に合わせる)
        no_load_speed = p('motor_no_load_speed', 0.0).value      # デューティ比 1.0 の速さ [rad/s]
        time_constant = p('motor_time_constant', 0.0).value      # 速さが追いつく速さ [s]
        mech_steer = p('mech_steer_ratio', 0.0).value             # 本当の機構のギア比
        mech_wheel = p('mech_wheel_ratio', 0.0).value
        self.timeout_s = p('timeout_s', 0.1).value      # app_config.h の APP_COMMAND_TIMEOUT_MS
        rate_hz = p('rate_hz', 100.0).value

        lib = firmware_sim.build(firmware_dir)
        self.sims = [firmware_sim.SwerveSim(lib, no_load_speed, time_constant, mech_steer,
                                            mech_wheel) for _ in self.ids]
        self.targets = [(0.0, 0.0, 0.0)] * len(self.ids)   # (受け取った時刻, 向き, 速さ)
        self.state_pubs = []
        self.true_pubs = []
        for i, dev_id in enumerate(self.ids):
            self.state_pubs.append(self.create_publisher(
                Float32MultiArray, f'stm{dev_id}/state', 10))
            self.true_pubs.append(self.create_publisher(
                Float32MultiArray, f'stm{dev_id}/true_state', 10))
            self.create_subscription(
                Float32MultiArray, f'stm{dev_id}/command',
                lambda msg, i=i: self.on_command(i, msg), 10)
        self.period = 1.0 / rate_hz
        self.create_timer(self.period, self.step)
        self.get_logger().info(
            f'ID {self.ids} のモジュールを、{firmware_dir} のファームウェアで動かしています')

    def on_command(self, index, msg):
        if len(msg.data) >= 2:
            self.targets[index] = (time.monotonic(), float(msg.data[0]), float(msg.data[1]))

    def step(self):
        now = time.monotonic()
        for i, (received, angle, speed) in enumerate(self.targets):
            running = now - received < self.timeout_s    # 指令が途切れたら止まる(ファームウェアと同じ)
            out = self.sims[i].run(angle, speed, running, self.period)
            self.state_pubs[i].publish(Float32MultiArray(data=out[:2]))
            self.true_pubs[i].publish(Float32MultiArray(data=out[2:]))

    def destroy_node(self):
        for sim in self.sims:
            sim.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = FirmwareModules()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
