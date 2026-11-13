"""
ロボットコンソール: 操縦・速さ・状態・停止を 1 つにまとめたウィンドウ(PC で動く).

    ┌ ROBOT CONSOLE ───────────────── [● すべて OK] ─ [■ 停止] ┐
    │ ロボット(上から見た図) │ 速さ(指令と実際) │ 軌跡(地図)  │
    │                        │ 操縦(WASD)       │              │
    │ 状態(/diagnostics の一覧)                                │
    └──────────────────────────────────────────────────────┘

    操縦 … W / S 前後、A / D 左右、Q / E 回転、Shift で速く。押している間だけ /cmd_vel を出す。
           ほかのウィンドウに移ると、押していないことにして止まる
    停止 … 停止ボタン、Esc、スペースキー。/drive_stop に true を出し、足回りは /cmd_vel を無視して止まる。
           「再開」を押すまで止まったまま

停止ボタンはソフトウェアの停止。**本物の非常停止スイッチ(モーターの電源を切るもの)は、必ず別に付けること。**

見るだけのトピック: /cmd_vel、/odom、swerve/markers、omni/markers、/diagnostics
(スワーブでもオムニでも、シミュレーションでも実機でも使える)

    ros2 run swerve_drive robot_console

ファイルの分担:
    console_state.py   … 表示するデータ(ROS にも画面にも依存しない。テストあり)
    console_theme.py   … 色とフォント
    console_widgets.py … 各カードの絵
    robot_console.py   … ROS とのやり取りと、ウィンドウ(このファイル)
"""

import math
import signal
import sys
import threading
import time
import traceback

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtWidgets import (QApplication, QGridLayout, QHBoxLayout, QLabel, QPushButton,
                             QSizePolicy, QWidget)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray

from . import console_state as cs
from . import console_theme as th
from . import console_widgets as widgets
from . import robot_status
from .teleop_keys import KEY_DIRECTIONS, twist_from_keys

QT_KEYS = {Qt.Key_W: 'w', Qt.Key_A: 'a', Qt.Key_S: 's', Qt.Key_D: 'd', Qt.Key_Q: 'q',
           Qt.Key_E: 'e', Qt.Key_Shift: 'shift'}


def yaw_of(orientation):
    """水平面で回っているだけの向き(クォータニオン)から、角度 [rad] を求める."""
    return 2.0 * math.atan2(orientation.z, orientation.w)


class ConsoleNode(Node):
    """ROS とのやり取り。受け取ったものは ConsoleState に入れる."""

    def __init__(self):
        super().__init__('robot_console')
        p = self.declare_parameter
        self.linear_speed = p('linear_speed', 0.5).value        # WASD の速さ [m/s]
        self.angular_speed = p('angular_speed', 1.5).value      # Q / E の速さ [rad/s]
        self.turbo_scale = p('turbo_scale', 2.0).value          # Shift で何倍にするか
        self.rate_hz = p('rate_hz', 20.0).value                 # /cmd_vel を出す回数 [回/秒]
        self.max_linear = p('max_linear', 1.0).value            # バーの端の速さ [m/s]
        self.max_angular = p('max_angular', 3.0).value          # [rad/s]
        self.state = cs.ConsoleState()
        self.was_moving = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.stop_pub = self.create_publisher(Bool, robot_status.STOP_TOPIC,
                                              robot_status.STOP_QOS)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        for drive in ('swerve', 'omni'):
            self.create_subscription(MarkerArray, f'{drive}/markers',
                                     lambda msg, d=drive: self.on_markers(d, msg), 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)
        self.create_subscription(Bool, robot_status.STOP_TOPIC, self.on_stop,
                                 robot_status.STOP_QOS)

    # ---- 受け取る(ROS のスレッド) ----

    def on_cmd_vel(self, msg):
        self.state.on_cmd(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())

    def on_odom(self, msg):
        pose, twist = msg.pose.pose, msg.twist.twist
        self.state.on_odom(pose.position.x, pose.position.y, yaw_of(pose.orientation),
                           twist.linear.x, twist.linear.y, twist.angular.z, time.monotonic())

    def on_markers(self, drive, msg):
        # 足回りのノードは、番号が偶数の矢印を「目標」、奇数を「実際」として出している
        targets, actuals = [], []
        for m in msg.markers:
            arrow = cs.Arrow(m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation),
                             m.scale.x)
            (actuals if m.id % 2 else targets).append(arrow)
        self.state.on_arrows(drive, targets, actuals, time.monotonic())

    def on_diagnostics(self, msg):
        now = time.monotonic()
        for status in msg.status:
            level = status.level
            level = int.from_bytes(level, 'little') if isinstance(level, bytes) else int(level)
            self.state.on_diagnostic(status.name, level, status.message,
                                     {v.key: v.value for v in status.values}, now)

    def on_stop(self, msg):
        self.state.set_stop(msg.data, time.monotonic())

    # ---- 出す(画面のスレッド) ----

    def publish_keys(self, keys, turbo):
        """押しているキーから /cmd_vel を出す。離したときは 1 回だけ 0 を出して止める."""
        twist = twist_from_keys(keys, self.linear_speed, self.angular_speed, turbo,
                                self.turbo_scale)
        moving = any(v != 0.0 for v in twist)
        if moving or self.was_moving:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = twist
            self.cmd_pub.publish(msg)
        self.was_moving = moving

    def publish_stop(self, stop):
        self.state.set_stop(stop, time.monotonic())
        self.stop_pub.publish(Bool(data=stop))


class ConsoleWindow(QWidget):
    """
    ウィンドウ。キー入力を受け取り、30 回/秒で表示を更新する.

    ウィンドウの大きさを変えると、中身をまるごと同じ比率で拡大・縮小する
    (BASE_SIZE のときが倍率 1。文字もボタンも余白も、同じ倍率になる)。
    """

    BASE_SIZE = (1240, 800)     # 倍率 1 のときのウィンドウの大きさ
    MIN_SCALE, MAX_SCALE = 0.7, 4.0
    SCALE_STEP = 0.05           # 倍率はこの刻みで変える(大きさを変えている間、がたがたしないように)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.keys = set()           # 押しているキー('w' など。Shift は 'shift')
        self.stopped = False
        self.header_key = None      # 見出しを作り直すかどうかの判定用
        self.k = 0.0                # 今の倍率(最初の apply_scale で決まる)

        self.setWindowTitle('ロボットコンソール')
        self.setObjectName('console')
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f'#console {{ background: {th.BG}; }}'
                           f' QLabel {{ color: {th.TEXT}; background: transparent; }}')
        self.setFocusPolicy(Qt.StrongFocus)
        self.resize(*self.BASE_SIZE)
        self.setMinimumSize(round(self.BASE_SIZE[0] * self.MIN_SCALE),
                            round(self.BASE_SIZE[1] * self.MIN_SCALE))

        self.robot = widgets.RobotView(node.max_linear, node.max_angular)
        self.speed = widgets.SpeedGauges(node.max_linear, node.max_angular)
        self.keypad = widgets.KeyPad(f'{node.linear_speed:g} m/s ・ Shift ×{node.turbo_scale:g}')
        self.keypad.keyChanged.connect(self.on_pad_key)
        self.trail = widgets.TrailView()
        self.status = widgets.StatusList()
        self.clear = QPushButton('消す')
        self.clear.setFocusPolicy(Qt.NoFocus)
        self.clear.setCursor(Qt.PointingHandCursor)
        self.clear.clicked.connect(node.state.clear_trail)
        self.note = QLabel('停止ボタンはソフトウェアの停止です。本物の非常停止スイッチ(電源を切るもの)は、必ず別に付けてください')
        self.note.setStyleSheet(f'color: {th.FAINT};')
        # 文が長いので、幅が足りないときは切れてよい(この文のせいでウィンドウを狭くできなくならないように)
        self.note.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.note.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.cards = [widgets.Card('ロボット', 'ROBOT', self.robot),
                      widgets.Card('速さ', 'SPEED', self.speed),
                      widgets.Card('操縦', 'DRIVE', self.keypad),
                      widgets.Card('軌跡', 'TRAIL', self.trail, self.clear),
                      widgets.Card('状態', 'STATUS', self.status, self.note)]
        self.cards[4].head.setStretchFactor(self.note, 1)
        # 中身の文字の長さで列の幅が変わらないように、幅は列の比(下の 5 : 4 : 5)だけで決める
        for card in self.cards:
            card.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.grid = QGridLayout(self)
        self.grid.addLayout(self.make_header(), 0, 0, 1, 3)
        self.grid.addWidget(self.cards[0], 1, 0, 2, 1)
        self.grid.addWidget(self.cards[1], 1, 1)
        self.grid.addWidget(self.cards[2], 2, 1)
        self.grid.addWidget(self.cards[3], 1, 2, 2, 1)
        self.grid.addWidget(self.cards[4], 3, 0, 1, 3)
        for column, stretch in enumerate((5, 4, 5)):
            self.grid.setColumnStretch(column, stretch)
        self.grid.setRowStretch(1, 1)
        self.grid.setRowStretch(2, 1)
        self.apply_scale(1.0)

        self.view_timer = QTimer(self)
        self.view_timer.timeout.connect(self.refresh)
        self.view_timer.start(33)
        self.key_timer = QTimer(self)
        self.key_timer.timeout.connect(self.send_keys)
        self.key_timer.start(int(1000 / node.rate_hz))
        self.refresh()

    def make_header(self):
        self.header = QHBoxLayout()
        self.logo = QLabel()
        self.title = QLabel('ROBOT CONSOLE')
        self.drive_chip = QLabel('')
        self.pill = QLabel('')
        self.pill.setAlignment(Qt.AlignCenter)
        self.stop_button = QPushButton('')
        self.stop_button.setFocusPolicy(Qt.NoFocus)     # スペースキーなどで勝手に押されないように
        self.stop_button.setCursor(Qt.PointingHandCursor)
        self.stop_button.clicked.connect(lambda: self.node.publish_stop(not self.stopped))
        self.header.addWidget(self.logo)
        self.header.addWidget(self.title)
        self.header.addWidget(self.drive_chip, 0, Qt.AlignVCenter)
        self.header.addStretch(1)
        self.header.addWidget(self.pill, 0, Qt.AlignVCenter)
        self.header.addWidget(self.stop_button)
        return self.header

    # ---- 大きさ ----

    def resizeEvent(self, event):
        super().resizeEvent(event)
        k = min(self.width() / self.BASE_SIZE[0], self.height() / self.BASE_SIZE[1])
        k = max(self.MIN_SCALE, min(self.MAX_SCALE, k))
        self.apply_scale(math.floor(k / self.SCALE_STEP + 1e-6) * self.SCALE_STEP)

    def apply_scale(self, k):
        """文字・ボタン・余白・各カードを、倍率 k の大きさにする."""
        if abs(k - self.k) < 1e-6:
            return
        self.k = k

        def px(value):
            return round(value * k)

        self.grid.setContentsMargins(px(18), px(14), px(18), px(16))
        self.grid.setSpacing(px(14))
        self.header.setSpacing(px(12))
        self.logo.setFixedSize(px(26), px(26))
        self.logo.setStyleSheet(
            f'background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 {th.ACCENT},'
            f' stop:1 {th.ACCENT_2}); border-radius: {px(8)}px;')
        self.title.setFont(th.font(14, bold=True, spacing=2.5, scale=k))
        self.drive_chip.setFont(th.font(8, bold=True, spacing=1.5, scale=k))
        self.pill.setFont(th.font(10, bold=True, scale=k))
        self.pill.setFixedHeight(px(34))
        self.stop_button.setFont(th.font(12, bold=True, scale=k))
        self.stop_button.setFixedSize(px(190), px(46))
        self.clear.setFont(th.font(9, scale=k))
        self.clear.setStyleSheet(
            f'QPushButton {{ color: {th.DIM}; background: {th.TRACK}; border: 1px solid {th.EDGE};'
            f' border-radius: {px(10)}px; padding: {px(2)}px {px(12)}px; }}'
            f'QPushButton:hover {{ color: {th.TEXT}; }}')
        self.note.setFont(th.font(8, scale=k))
        for card in self.cards:
            card.set_scale(k)
        self.header_key = None      # 見出しの色付きの部品も、新しい大きさで作り直す
        if self.isVisible():
            self.update_header(self.node.state.snapshot(time.monotonic()))

    # ---- 表示 ----

    def refresh(self):
        if not rclpy.ok():
            self.close()
            return
        snap = self.node.state.snapshot(time.monotonic())
        self.stopped = snap.stopped
        for view in (self.robot, self.speed, self.trail, self.status):
            view.set_snapshot(snap)
        self.keypad.set_keys(self.keys, self.isActiveWindow(), snap.stopped)
        self.update_header(snap)

    def update_header(self, snap):
        key = (snap.level, snap.summary, snap.stopped, snap.drive)
        if key == self.header_key:      # 変わったときだけ作り直す
            return
        self.header_key = key

        def px(value):
            return round(value * self.k)

        hex_color = th.LEVEL_COLOR[snap.level]
        self.pill.setText(f'●  {snap.summary}')
        self.pill.setStyleSheet(
            f'color: {hex_color}; background: {th.rgba(hex_color, 34)};'
            f' border: 1px solid {th.rgba(hex_color, 110)}; border-radius: {px(34) // 2 - 1}px;'
            f' padding: 0px {px(16)}px;')
        names = {'swerve': 'SWERVE  スワーブ', 'omni': 'OMNI  オムニ'}
        self.drive_chip.setText(names.get(snap.drive, ''))
        self.drive_chip.setStyleSheet(
            f'color: {th.ACCENT}; background: {th.rgba(th.ACCENT, 26)};'
            f' border-radius: {px(9)}px; padding: {px(3)}px {px(10)}px;'
            if snap.drive else 'background: transparent;')
        if snap.stopped:
            base, hover, text, label = th.GOOD, '#5fe3b4', '#04130d', '▶  再開'
        else:
            base, hover, text, label = th.BAD, '#ff7a8c', '#ffffff', '■  停止(Esc)'
        self.stop_button.setText(label)
        self.stop_button.setStyleSheet(
            f'QPushButton {{ background: {base}; color: {text}; border: none;'
            f' border-radius: {px(14)}px; }} QPushButton:hover {{ background: {hover}; }}')

    # ---- キー ----

    def keyPressEvent(self, event):
        if event.isAutoRepeat():        # 押しっぱなしのときに OS が繰り返す分は無視する
            return
        if event.key() in (Qt.Key_Escape, Qt.Key_Space):
            self.node.publish_stop(True)
        elif event.key() in QT_KEYS:
            self.keys.add(QT_KEYS[event.key()])

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat() and event.key() in QT_KEYS:
            self.keys.discard(QT_KEYS[event.key()])

    def on_pad_key(self, name, pressed):
        """画面のキーをマウスで押した / 離した."""
        (self.keys.add if pressed else self.keys.discard)(name)

    def changeEvent(self, event):
        # ほかのウィンドウに移ったら、押していないことにする(離したことが届かないため)
        if event.type() == QEvent.ActivationChange and not self.isActiveWindow():
            self.keys.clear()
        super().changeEvent(event)

    def send_keys(self):
        moving = {k for k in self.keys if k in KEY_DIRECTIONS}
        self.node.publish_keys(moving, 'shift' in self.keys)

    def closeEvent(self, event):
        self.keys.clear()
        self.node.publish_keys(set(), False)    # 閉じるときは止める
        super().closeEvent(event)


def spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:   # 終了するとき
        pass


def main():
    rclpy.init()
    app = QApplication(sys.argv[:1])
    app.setFont(th.font(10))
    # 画面の処理の中でエラーが起きても、アプリごと落ちないようにする(内容は表示する)
    sys.excepthook = lambda *error: traceback.print_exception(*error)
    node = ConsoleNode()
    window = ConsoleWindow(node)
    signal.signal(signal.SIGINT, lambda *_: app.quit())     # ターミナルの Ctrl+C で閉じる
    thread = threading.Thread(target=spin, args=(node,), daemon=True)
    thread.start()
    window.show()
    app.exec_()
    node.publish_keys(set(), False)     # 走っている途中で閉じたときは、止める
    rclpy.try_shutdown()
    thread.join(timeout=1.0)
    node.destroy_node()


if __name__ == '__main__':
    main()
