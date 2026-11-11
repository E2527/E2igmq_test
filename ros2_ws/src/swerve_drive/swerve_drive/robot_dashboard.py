"""
ロボットの状態ダッシュボード(PC で動く小さなウィンドウ).

    ・/diagnostics を一覧で表示する(stm_bridge、足回りなど、各ノードが出す「OK / 注意 / エラー」)
      しばらく届かない項目は灰色で「届いていない」と出す(そのノードが止まったか、Wi-Fi が切れた)
    ・停止ボタン: 押すと /drive_stop に true を出し、足回りは /cmd_vel を無視して止まる。
      もう一度押す(再開)まで止まったまま。Esc キーでも止められる(再開はボタンだけ)

停止ボタンはソフトウェアの停止。**本物の非常停止スイッチ(電源を切るもの)は、必ず別に付けること。**

同じ /diagnostics は rqt_robot_monitor でも見られる(ros2 run rqt_robot_monitor rqt_robot_monitor)。

    ros2 run swerve_drive robot_dashboard
"""

import time
import tkinter as tk

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from . import robot_status

STALE_S = 3.0       # これより長く届かない項目は「届いていない」にする
COLORS = {robot_status.OK: 'green3', robot_status.WARN: 'orange', robot_status.ERROR: 'red2'}
LEVEL_TEXT = {robot_status.OK: 'OK', robot_status.WARN: '注意', robot_status.ERROR: 'エラー'}


def level_of(status):
    """状態の重さ(OK / 注意 / エラー)を整数で返す。bytes で届くことがあるのでそろえる."""
    level = status.level
    return int.from_bytes(level, 'little') if isinstance(level, bytes) else int(level)


def summarize(items, now):
    """項目の一覧(名前 → (受け取った時刻, status))から、全体の色と文を決める."""
    if not items:
        return 'gray50', '/diagnostics が届いていません(足回りや stm_bridge が起動しているか確認)'
    # しばらく届いていない項目は、エラーと同じ扱いにする
    worst = max(robot_status.ERROR if now - received > STALE_S else level_of(status)
                for received, status in items.values())
    if worst >= robot_status.ERROR:
        return COLORS[robot_status.ERROR], 'エラーがあります'
    if worst >= robot_status.WARN:
        return COLORS[robot_status.WARN], '注意があります'
    return COLORS[robot_status.OK], 'すべて OK'


class RobotDashboard(Node):

    def __init__(self):
        super().__init__('robot_dashboard')
        self.items = {}         # 名前 → (受け取った時刻, DiagnosticStatus)
        self.stopped = False
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)
        self.create_subscription(Bool, robot_status.STOP_TOPIC, self.on_stop,
                                 robot_status.STOP_QOS)
        self.stop_pub = self.create_publisher(Bool, robot_status.STOP_TOPIC,
                                              robot_status.STOP_QOS)

        self.root = tk.Tk()
        self.root.title('ロボットの状態')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.bind('<Escape>', lambda _: self.set_stop(True))
        top = tk.Frame(self.root, padx=12, pady=10)
        top.pack(fill='x')
        self.summary = tk.Label(top, font=('sans', 14, 'bold'), anchor='w')
        self.summary.pack(side='left', fill='x', expand=True)
        self.stop_button = tk.Button(top, font=('sans', 14, 'bold'), width=8,
                                     command=lambda: self.set_stop(not self.stopped))
        self.stop_button.pack(side='right')
        self.table = tk.Frame(self.root, padx=12, pady=4)
        self.table.pack(fill='both', expand=True)
        tk.Label(self.root, fg='gray40', padx=12, pady=6, anchor='w', justify='left',
                 text='停止ボタンはソフトウェアの停止です(Esc でも止まる)。'
                      '本物の非常停止スイッチは別に付けてください').pack(fill='x')
        self.rows = {}
        self.running = True
        self.root.after(50, self.tick)

    # ---- 受け取る ----

    def on_diagnostics(self, msg):
        now = time.monotonic()
        for status in msg.status:
            self.items[status.name] = (now, status)

    def on_stop(self, msg):
        self.stopped = msg.data

    def set_stop(self, stop):
        self.stopped = stop
        self.stop_pub.publish(Bool(data=stop))

    # ---- 表示 ----

    def tick(self):
        if not self.running:
            return
        if not rclpy.ok():                  # ターミナルで Ctrl+C を押したとき
            self.running = False
            self.root.destroy()
            return
        rclpy.spin_once(self, timeout_sec=0.0)
        now = time.monotonic()
        color, text = summarize(self.items, now)
        if self.stopped:
            color, text = COLORS[robot_status.WARN], '停止中(停止ボタン)'
        self.summary.config(text=text, fg=color)
        self.stop_button.config(text='▶ 再開' if self.stopped else '■ 停止',
                                bg='green3' if self.stopped else 'red2', fg='white',
                                activebackground='green4' if self.stopped else 'red3')
        for name in sorted(self.items):
            self.show_row(name, *self.items[name], now)
        self.root.after(200, self.tick)

    def show_row(self, name, received, status, now):
        if name not in self.rows:
            r = len(self.rows)
            dot = tk.Canvas(self.table, width=14, height=14, highlightthickness=0)
            dot.grid(row=r, column=0, padx=(0, 6), pady=2)
            labels = [tk.Label(self.table, anchor='w', font=font)
                      for font in (('sans', 11, 'bold'), ('sans', 11), ('monospace', 10))]
            for c, label in enumerate(labels, start=1):
                label.grid(row=r, column=c, sticky='w', padx=6)
            self.rows[name] = (dot, labels)
        dot, (name_label, message_label, values_label) = self.rows[name]
        if now - received > STALE_S:
            color = 'gray60'
            message = f'届いていない({now - received:.0f} 秒前が最後)'
        else:
            level = level_of(status)
            color = COLORS.get(level, 'gray60')
            message = f'{LEVEL_TEXT.get(level, "?")}: {status.message}'
        dot.delete('all')
        dot.create_oval(2, 2, 12, 12, fill=color, outline='')
        name_label.config(text=name)
        message_label.config(text=message)
        values_label.config(text='  '.join(f'{v.key} {v.value}' for v in status.values))

    def close(self):
        self.running = False
        self.root.destroy()


def main():
    rclpy.init()
    node = RobotDashboard()
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
