"""
指令の速さと実際の速さを並べて表示する小さなウィンドウ.

    指令 … /cmd_vel(WASD・ゲームパッド・Nav2 が出す、「この速さで走って」という値)
    実際 … /odom(足回りが、タイヤの実際の回転から計算した速さ)

前後・左右・回転を、数字と棒グラフ(薄い色が指令、濃い色が実際)で表示する。
実際が指令より大きく遅いときは「頭打ち」と表示する(タイヤやモーターの上限に当たっている)。
トピックを見るだけなので、スワーブでもオムニでも、シミュレーションでも実機でも使える。

    ros2 run swerve_drive drive_monitor
"""

import time
import tkinter as tk

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

# (名前, 単位, 指令/実際から値を取り出す番号, 棒グラフの端の値のパラメータ)
ROWS = [('前後', 'm/s', 0, 'max_linear'),
        ('左右', 'm/s', 1, 'max_linear'),
        ('回転', 'rad/s', 2, 'max_angular')]
BAR_WIDTH = 260
BAR_HEIGHT = 14


class DriveMonitor(Node):

    def __init__(self):
        super().__init__('drive_monitor')
        self.limits = {
            'max_linear': self.declare_parameter('max_linear', 1.5).value,     # 棒グラフの端 [m/s]
            'max_angular': self.declare_parameter('max_angular', 3.0).value,   # [rad/s]
        }
        self.cmd_timeout_s = self.declare_parameter('cmd_timeout_s', 0.5).value
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None
        self.actual = (0.0, 0.0, 0.0)
        self.actual_time = None
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)

        self.root = tk.Tk()
        self.root.title('速さモニター')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        frame = tk.Frame(self.root, padx=12, pady=12)
        frame.pack()
        self.labels = []
        self.bars = []
        for r, (name, _, _, _) in enumerate(ROWS):
            tk.Label(frame, text=name, font=('sans', 12)).grid(row=r, column=0, sticky='w')
            canvas = tk.Canvas(frame, width=BAR_WIDTH, height=BAR_HEIGHT * 2 + 4,
                               highlightthickness=0)
            canvas.grid(row=r, column=1, padx=8, pady=4)
            label = tk.Label(frame, font=('monospace', 11), justify='left', width=30, anchor='w')
            label.grid(row=r, column=2, sticky='w')
            self.bars.append(canvas)
            self.labels.append(label)
        self.status = tk.Label(frame, font=('sans', 11))
        self.status.grid(row=len(ROWS), column=0, columnspan=3, sticky='w', pady=(8, 0))
        tk.Label(frame, text='棒グラフ: 薄い色 = 指令(/cmd_vel)、濃い色 = 実際(/odom)',
                 fg='gray40').grid(row=len(ROWS) + 1, column=0, columnspan=3, sticky='w')
        self.running = True
        self.root.after(20, self.tick)

    def on_cmd_vel(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.cmd_time = time.monotonic()

    def on_odom(self, msg):
        t = msg.twist.twist
        self.actual = (t.linear.x, t.linear.y, t.angular.z)
        self.actual_time = time.monotonic()

    def tick(self):
        if not self.running:
            return
        if not rclpy.ok():                  # ターミナルで Ctrl+C を押したとき
            self.running = False
            self.root.destroy()
            return
        rclpy.spin_once(self, timeout_sec=0.0)
        now = time.monotonic()
        # 指令が途切れたら、足回りは止まるので、指令も 0 として表示する
        cmd_fresh = self.cmd_time is not None and now - self.cmd_time < self.cmd_timeout_s
        cmd = self.cmd if cmd_fresh else (0.0, 0.0, 0.0)
        odom_fresh = self.actual_time is not None and now - self.actual_time < 1.0
        actual = self.actual if odom_fresh else None
        for (_, unit, i, limit), canvas, label in zip(ROWS, self.bars, self.labels):
            self.draw_bar(canvas, cmd[i], actual[i] if actual else 0.0, self.limits[limit])
            text = f'指令 {cmd[i]:+.2f}  実際 '
            text += f'{actual[i]:+.2f} {unit}' if actual else '  --  ' + unit
            label.config(text=text)
        self.status.config(**self.status_text(cmd, actual))
        self.root.after(20, self.tick)

    @staticmethod
    def status_text(cmd, actual):
        if actual is None:
            return {'text': '/odom が届いていません(足回りが起動しているか確認)', 'fg': 'orange3'}
        wanted = max(abs(v) for v in cmd)
        if wanted < 1e-3:
            return {'text': '停止中', 'fg': 'gray40'}
        # どれか 1 つでも、実際が指令の 8 割に届いていなければ「頭打ち」
        slow = any(abs(c) > 0.05 and abs(a) < 0.8 * abs(c) for c, a in zip(cmd, actual))
        if slow:
            return {'text': '頭打ち: 実際が指令より遅い(タイヤやモーターの上限、加速中)',
                    'fg': 'red3'}
        return {'text': '指令どおりに走っています', 'fg': 'green4'}

    @staticmethod
    def draw_bar(canvas, cmd, actual, limit):
        canvas.delete('all')
        mid = BAR_WIDTH / 2

        def x(v):
            return mid + max(-1.0, min(1.0, v / limit)) * (mid - 2)

        canvas.create_rectangle(0, 0, BAR_WIDTH, BAR_HEIGHT * 2 + 4, fill='gray92', width=0)
        canvas.create_rectangle(min(mid, x(cmd)), 2, max(mid, x(cmd)), 2 + BAR_HEIGHT,
                                fill='#9ec5f0', width=0)
        canvas.create_rectangle(min(mid, x(actual)), 2 + BAR_HEIGHT, max(mid, x(actual)),
                                2 + BAR_HEIGHT * 2, fill='#1f6fbf', width=0)
        canvas.create_line(mid, 0, mid, BAR_HEIGHT * 2 + 4, fill='gray50')

    def close(self):
        self.running = False
        self.root.destroy()


def main():
    rclpy.init()
    node = DriveMonitor()
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
