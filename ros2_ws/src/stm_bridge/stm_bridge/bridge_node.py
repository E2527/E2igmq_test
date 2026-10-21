"""
CAN バスにつないだ複数の STM32 と ROS 2 トピックを中継するノード.

  stm<ID>/command (std_msgs/Float32MultiArray) … モーター 2 個の目標速度 [rad/s]
  stm<ID>/state   (std_msgs/Float32MultiArray) … モーター 2 個の実際の速度 [rad/s]

デューティ比は STM32 がエンコーダで測った速度から PID で決める(firmware/stm32_can/motor.c)。

指令は受け取った最新の値を rate_hz の周期で送り続ける(STM32 側はこれが途切れたら止まる)。
command が command_timeout_s 以上届かないときは空の指令(= 止まる)を送る。
状態は STM32 が自分の周期で送ってくるので、届いたらそのまま publish する。
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from stm_bridge import can_bus, protocol


class StmBridge(Node):

    def __init__(self):
        super().__init__('stm_bridge')
        self.interface = self.declare_parameter('can_interface', 'can0').value
        self.device_ids = list(self.declare_parameter('device_ids', [1]).value)
        rate_hz = self.declare_parameter('rate_hz', 100.0).value
        self.command_timeout_s = self.declare_parameter('command_timeout_s', 0.5).value
        self.state_timeout_s = self.declare_parameter('state_timeout_s', 0.5).value
        for dev_id in self.device_ids:
            protocol.command_id(dev_id)  # ID の範囲を確認

        self.bus = None
        self.bus_lock = threading.Lock()
        self.next_open_time = 0.0
        self.commands = {}        # ID -> (受け取った時刻, data)
        self.last_state = {}      # ID -> 最後に状態が届いた時刻
        self.rx_count = {i: 0 for i in self.device_ids}
        self.tx_errors = 0

        self.state_pubs = {}
        for dev_id in self.device_ids:
            self.state_pubs[dev_id] = self.create_publisher(
                Float32MultiArray, f'stm{dev_id}/state', 10)
            self.create_subscription(
                Float32MultiArray, f'stm{dev_id}/command',
                lambda msg, i=dev_id: self.on_command(i, msg), 10)

        self.create_timer(1.0 / rate_hz, self.send_commands)
        self.create_timer(5.0, self.report)
        self.running = True
        self.rx_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.rx_thread.start()
        self.get_logger().info(
            f'can_interface={self.interface} device_ids={self.device_ids} rate={rate_hz}Hz')

    def on_command(self, dev_id, msg):
        try:
            data = protocol.pack_floats(msg.data)
        except ValueError as e:
            self.get_logger().error(f'stm{dev_id}/command: {e}')
            return
        self.commands[dev_id] = (time.monotonic(), data)

    def open_bus(self):
        now = time.monotonic()
        if now < self.next_open_time:
            return None
        try:
            bus = can_bus.open_bus(self.interface, protocol.STATE_FILTER)
        except (OSError, ValueError) as e:
            self.next_open_time = now + 1.0
            self.get_logger().warn(
                f'{self.interface} を開けません: {e}'
                f'(CAN なら can0 が up か、シリアルなら USB がつながっているか確認してください)',
                throttle_duration_sec=5.0)
            return None
        self.get_logger().info(f'{self.interface} を開きました')
        with self.bus_lock:
            self.bus = bus
        return bus

    def close_bus(self, reason):
        with self.bus_lock:
            if self.bus is None:
                return
            self.bus.close()
            self.bus = None
        self.next_open_time = time.monotonic() + 1.0
        self.get_logger().error(f'CAN 通信エラー: {reason}。1 秒後に開き直します')

    def send_commands(self):
        bus = self.bus or self.open_bus()
        if bus is None:
            return
        now = time.monotonic()
        for dev_id in self.device_ids:
            data = b''
            received = self.commands.get(dev_id)
            if received and now - received[0] < self.command_timeout_s:
                data = received[1]
            try:
                bus.send(protocol.command_id(dev_id), data)
            except OSError as e:
                if e.errno in can_bus.TRANSIENT_ERRORS:
                    self.tx_errors += 1
                    continue
                self.close_bus(e)
                return

    def receive_loop(self):
        while self.running:
            bus = self.bus
            if bus is None:
                time.sleep(0.1)
                continue
            try:
                frame = bus.recv(0.2)
            except OSError as e:
                if self.running:
                    self.close_bus(e)
                continue
            if frame is None:
                continue
            dev_id = protocol.device_of(frame[0], protocol.STATE_BASE)
            if dev_id not in self.state_pubs:
                continue
            try:
                values = protocol.unpack_floats(frame[1])
            except ValueError as e:
                self.get_logger().warn(f'stm{dev_id}: {e}', throttle_duration_sec=5.0)
                continue
            self.last_state[dev_id] = time.monotonic()
            self.rx_count[dev_id] += 1
            self.state_pubs[dev_id].publish(Float32MultiArray(data=values))

    def report(self):
        if self.bus is None:
            return
        now = time.monotonic()
        parts = [f'stm{i}: {n}' for i, n in self.rx_count.items()]
        msg = '状態の受信数(5 秒間) ' + ', '.join(parts)
        if self.tx_errors:
            msg += f'、送信失敗 {self.tx_errors}(バスに他の基板がいない / 配線・終端抵抗を確認)'
        self.get_logger().info(msg)
        for dev_id in self.device_ids:
            last = self.last_state.get(dev_id)
            if last is None or now - last > self.state_timeout_s:
                self.get_logger().warn(f'stm{dev_id} から状態が届いていません')
            self.rx_count[dev_id] = 0
        self.tx_errors = 0

    def shutdown(self):
        self.running = False
        self.rx_thread.join(timeout=1.0)
        with self.bus_lock:
            if self.bus is not None:
                self.bus.close()
                self.bus = None


def main():
    rclpy.init()
    node = StmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
