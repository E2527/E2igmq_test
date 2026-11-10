"""
WASD でロボットを操縦するノード(小さなウィンドウを開いて、キーを読む).

    ros2 run swerve_drive keyboard_teleop

    W / S … 前 / 後ろ      A / D … 左 / 右(向きを変えずに横に動く)
    Q / E … 左回り / 右回り  Shift … 速くする

・キーを押している間だけ /cmd_vel を出す(離すと止まる)。W と D を同時に押すと斜めに進む
・ウィンドウからほかのウィンドウに移ると、押していないことにして止まる(誤操作の防止)
・ターミナルではなくウィンドウでキーを読むのは、ターミナルだと「離した」が分からず、
  2 つのキーの同時押しも読めないため
"""

import tkinter as tk

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from .teleop_keys import KEY_DIRECTIONS, twist_from_keys

# キーを押し続けると、OS が「離した・押した」を自動で繰り返す(オートリピート)。
# 離してからこの時間 [ms] のうちにまた押されたら、離していないとみなす
RELEASE_DELAY_MS = 60
SHIFT_KEYS = ('Shift_L', 'Shift_R')


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.linear_speed = self.declare_parameter('linear_speed', 0.5).value     # [m/s]
        self.angular_speed = self.declare_parameter('angular_speed', 1.5).value   # [rad/s]
        self.turbo_scale = self.declare_parameter('turbo_scale', 2.0).value
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.keys = set()           # 今押しているキー(w a s d q e)
        self.turbo = False
        self.pending_release = {}   # キー → 「離した」を確定する予約(オートリピート対策)
        self.was_moving = False

        self.root = tk.Tk()
        self.root.title('WASD で操縦')
        self.label = tk.Label(self.root, font=('monospace', 14), justify='left', padx=16, pady=16)
        self.label.pack()
        self.root.bind('<KeyPress>', self.on_press)
        self.root.bind('<KeyRelease>', self.on_release)
        self.root.bind('<FocusOut>', lambda _: self.release_all())
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.period_ms = int(1000 / rate_hz)
        self.running = True
        self.root.after(self.period_ms, self.tick)
        self.show((0.0, 0.0, 0.0))

    # ---- キー ----

    def on_press(self, event):
        key = self.key_of(event)
        if key is None:
            return
        if key in self.pending_release:
            self.root.after_cancel(self.pending_release.pop(key))
        if key == 'shift':
            self.turbo = True
        else:
            self.keys.add(key)

    def on_release(self, event):
        key = self.key_of(event)
        if key is None:
            return
        self.pending_release[key] = self.root.after(RELEASE_DELAY_MS, self.release, key)

    def release(self, key):
        self.pending_release.pop(key, None)
        if key == 'shift':
            self.turbo = False
        else:
            self.keys.discard(key)

    def release_all(self):
        for after_id in self.pending_release.values():
            self.root.after_cancel(after_id)
        self.pending_release.clear()
        self.keys.clear()
        self.turbo = False

    @staticmethod
    def key_of(event):
        if event.keysym in SHIFT_KEYS:
            return 'shift'
        key = event.keysym.lower()
        return key if key in KEY_DIRECTIONS else None

    # ---- 周期処理 ----

    def tick(self):
        if not self.running:
            return
        if not rclpy.ok():                  # ターミナルで Ctrl+C を押したとき
            self.running = False
            self.root.destroy()
            return
        twist = twist_from_keys(self.keys, self.linear_speed, self.angular_speed,
                                self.turbo, self.turbo_scale)
        moving = any(v != 0.0 for v in twist)
        if moving or self.was_moving:       # 離したときは 1 回だけ 0 を送って止める
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = twist
            self.pub.publish(msg)
        self.was_moving = moving
        self.show(twist)
        self.root.after(self.period_ms, self.tick)

    def show(self, twist):
        vx, vy, omega = twist
        self.label.config(text=(
            'このウィンドウを選んだまま操作します\n\n'
            '  W / S   前 / 後ろ\n'
            '  A / D   左 / 右\n'
            '  Q / E   左回り / 右回り\n'
            '  Shift   速く\n\n'
            f'前 {vx:+.2f} m/s  左 {vy:+.2f} m/s  回転 {omega:+.2f} rad/s'))

    def close(self):
        self.running = False
        self.pub.publish(Twist())     # 閉じるときは止める
        self.root.destroy()


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
