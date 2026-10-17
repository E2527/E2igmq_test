"""
シリアルバス(RS-485 など)につないだ複数の STM32 と ROS 2 トピックを中継するノード.

Pi が親機になり、決まった周期で device_ids の STM32 に順番に問い合わせる(ポーリング)。

  stm<ID>/command (std_msgs/Float32MultiArray) … STM32 に送る指令値。受け取った最新の値を毎周期送る
  stm<ID>/state   (std_msgs/Float32MultiArray) … STM32 から返ってきた状態

command が command_timeout_s 以上届かないときは空の指令を送る。
STM32 側は「空の指令 = 止まる」として扱うことで、Pi 側が止まってもロボットが暴走しない。
"""

import time

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32MultiArray

from stm_bridge import protocol


class StmBridge(Node):

    def __init__(self):
        super().__init__('stm_bridge')
        self.port = self.declare_parameter('port', '/dev/stm_bus').value
        self.baudrate = self.declare_parameter('baudrate', 1000000).value
        self.device_ids = list(self.declare_parameter('device_ids', [1]).value)
        rate_hz = self.declare_parameter('rate_hz', 100.0).value
        # 返信を待つ時間 = 返信フレームの送信時間 + この余裕
        self.reply_margin_s = self.declare_parameter('reply_timeout_ms', 2.0).value / 1000.0
        self.command_timeout_s = self.declare_parameter('command_timeout_s', 0.5).value

        self.period_s = 1.0 / rate_hz
        self.frame_time_s = protocol.MAX_FRAME_SIZE * 10 / self.baudrate  # 1 バイト = 10 ビット
        self.serial = None
        self.parser = protocol.FrameParser()
        self.next_open_time = 0.0

        self.commands = {}        # ID -> (受け取った時刻, payload)
        self.stats = {i: {'ok': 0, 'timeout': 0} for i in self.device_ids}
        self.state_pubs = {}
        for dev_id in self.device_ids:
            self.state_pubs[dev_id] = self.create_publisher(
                Float32MultiArray, f'stm{dev_id}/state', 10)
            self.create_subscription(
                Float32MultiArray, f'stm{dev_id}/command',
                lambda msg, i=dev_id: self.on_command(i, msg), 10)

        self.create_timer(self.period_s, self.poll_all)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f'port={self.port} baudrate={self.baudrate} device_ids={self.device_ids} '
            f'rate={rate_hz}Hz')

    def on_command(self, dev_id, msg):
        try:
            payload = protocol.pack_floats(msg.data)
            protocol.encode(dev_id, protocol.CMD_DATA, payload)  # 長さの確認
        except ValueError as e:
            self.get_logger().error(f'stm{dev_id}/command: {e}')
            return
        self.commands[dev_id] = (time.monotonic(), payload)

    def open_serial(self):
        now = time.monotonic()
        if now < self.next_open_time:
            return False
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0)
            self.parser.reset()
            self.get_logger().info(f'{self.port} を開きました')
            return True
        except serial.SerialException as e:
            self.serial = None
            self.next_open_time = now + 1.0
            self.get_logger().warn(f'{self.port} を開けません: {e}', throttle_duration_sec=5.0)
            return False

    def poll_all(self):
        if self.serial is None and not self.open_serial():
            return
        start = time.monotonic()
        try:
            for dev_id in self.device_ids:
                self.poll(dev_id)
        except serial.SerialException as e:
            # USB が抜けたときなど。閉じて、1 秒後に開き直す
            self.get_logger().error(f'シリアル通信エラー: {e}')
            self.serial.close()
            self.serial = None
            self.next_open_time = time.monotonic() + 1.0
            return
        elapsed = time.monotonic() - start
        if elapsed > self.period_s:
            self.get_logger().warn(
                f'1 周期の通信に {elapsed * 1000:.1f} ms かかり、周期 {self.period_s * 1000:.1f} ms '
                'を超えました。rate_hz を下げるか baudrate を上げてください',
                throttle_duration_sec=5.0)

    def poll(self, dev_id):
        payload = b''
        received = self.commands.get(dev_id)
        if received and time.monotonic() - received[0] < self.command_timeout_s:
            payload = received[1]

        self.serial.reset_input_buffer()
        self.parser.reset()
        tx = protocol.encode(dev_id, protocol.CMD_DATA, payload)
        self.serial.write(tx)
        self.serial.flush()

        reply = self.wait_reply(dev_id, protocol.CMD_DATA | protocol.REPLY_FLAG)
        if reply is None:
            self.stats[dev_id]['timeout'] += 1
            return
        try:
            values = protocol.unpack_floats(reply.payload)
        except ValueError as e:
            self.get_logger().warn(f'stm{dev_id}: {e}', throttle_duration_sec=5.0)
            return
        self.stats[dev_id]['ok'] += 1
        self.state_pubs[dev_id].publish(Float32MultiArray(data=values))

    def wait_reply(self, dev_id, cmd):
        deadline = time.monotonic() + self.frame_time_s + self.reply_margin_s
        while time.monotonic() < deadline:
            data = self.serial.read(self.serial.in_waiting or 1)
            if not data:
                time.sleep(0.0001)
                continue
            for frame in self.parser.feed(data):
                # 自分が送ったフレームのエコーや、他の ID 宛ての返信は無視する
                if frame.dev_id == dev_id and frame.cmd == cmd:
                    return frame
        return None

    def report(self):
        if self.serial is None:
            return
        parts = [f'stm{i}: ok={s["ok"]} timeout={s["timeout"]}' for i, s in self.stats.items()]
        self.get_logger().info(
            '通信状況(5 秒間) ' + ', '.join(parts) + f', crc_error={self.parser.crc_errors}')
        for s in self.stats.values():
            s['ok'] = s['timeout'] = 0
        self.parser.crc_errors = 0


def main():
    rclpy.init()
    node = StmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial is not None:
            node.serial.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
