"""
ロボットコンソール: 操縦・速さ・状態・停止を 1 つにまとめたウィンドウ(PC で動く).

    ┌ [■ 停止] 状態 ● すべて OK │ 足回り スワーブ │ 軌跡を消す ──────────┐
    │                                     │ [操縦] [ロボット] [状態]  │
    │          軌跡(地図)               │  速さ(指令と実際)        │
    │   ロボットの位置・向き・走った跡    │  WASD のキー              │
    │                                     │                          │
    ├ x +0.00 m  y +0.00 m  向き +0°  速さ 0.00 m/s ─────────────────┤
    (フィールドエディタ・設定画面と同じ、道具バー + 中央の図 + 右の欄 の並び)

    操縦 … W / S 前後、A / D 左右、Q / E 回転、Shift で速く。押している間だけ /cmd_vel を出す。
           ほかのウィンドウに移ると、押していないことにして止まる
    停止 … 停止ボタン、Esc、スペースキー。/drive_stop に true を出し、足回りは /cmd_vel を無視して止まる。
           「再開」を押すまで止まったまま

停止ボタンはソフトウェアの停止。**本物の非常停止スイッチ(モーターの電源を切るもの)は、必ず別に付けること。**

見るだけのトピック: /cmd_vel、/odom、swerve/markers、omni/markers、/diagnostics、
                   /map(地図)、/scan(LiDAR)、TF(map → odom、base_link → LiDAR)
(スワーブでもオムニでも、シミュレーションでも実機でも使える。地図や LiDAR がなければ、その表示が出ないだけ)

    ros2 run swerve_drive robot_console

ファイルの分担:
    console_state.py   … 表示するデータ(ROS にも画面にも依存しない。テストあり)
    console_theme.py   … 色とフォント
    console_widgets.py … 各部品の絵(軌跡、ロボット、速さ、キー、状態)
    robot_console.py   … ROS とのやり取りと、ウィンドウ(このファイル)
"""

import math
import re
import signal
import sys
import threading
import time
import traceback

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtWidgets import (QAction, QApplication, QComboBox, QGroupBox, QHBoxLayout, QLabel,
                             QMainWindow, QPushButton, QTabWidget, QToolBar, QToolButton,
                             QVBoxLayout, QWidget)
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from . import console_state as cs
from . import console_theme as th
from . import console_widgets as widgets
from . import robot_status
from .teleop_keys import KEY_DIRECTIONS, twist_from_keys

QT_KEYS = {Qt.Key_W: 'w', Qt.Key_A: 'a', Qt.Key_S: 's', Qt.Key_D: 'd', Qt.Key_Q: 'q',
           Qt.Key_E: 'e', Qt.Key_Shift: 'shift'}


STM_TOPIC = re.compile(r'^/stm(\d+)/(command|state)$')
MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


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
        # 地図と LiDAR(なければ、その表示が出ないだけ)
        self.map_frame = p('map_frame', 'map').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_link').value
        self.create_subscription(OccupancyGrid, p('map_topic', 'map').value, self.on_map, MAP_QOS)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self.update_odom_in_map)
        # 通信の様子: stm<ID>/command と stm<ID>/state を見つけしだい、見るだけの購読をする
        self.stm_subscribed = set()
        self.create_timer(1.0, self.discover_stm)

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

    def on_map(self, msg):
        info = msg.info
        self.state.on_map(info.width, info.height, info.resolution, info.origin.position.x,
                          info.origin.position.y, msg.data)

    def lookup(self, target, source):
        """TF で source の座標が target のどこにあるか (x, y, 向き)。分からなければ None."""
        try:
            t = self.tf_buffer.lookup_transform(target, source, Time(),
                                                timeout=Duration(seconds=0.0))
        except TransformException:
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    def on_scan(self, msg):
        # LiDAR の取り付け位置(TF の base_link → LiDAR)。分からなければ、ロボットの中心で前向き
        laser = self.lookup(self.base_frame, msg.header.frame_id) or (0.0, 0.0, 0.0)
        points = cs.scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                msg.range_min, msg.range_max, laser)
        self.state.on_scan(points, time.monotonic())

    def discover_stm(self):
        for name, _types in self.get_topic_names_and_types():
            match = STM_TOPIC.match(name)
            if match is None or name in self.stm_subscribed:
                continue
            self.stm_subscribed.add(name)
            dev_id, kind = int(match.group(1)), match.group(2)
            self.create_subscription(
                Float32MultiArray, name,
                lambda msg, d=dev_id, k=kind: self.state.on_stm(d, k, msg.data, time.monotonic()),
                10)

    def update_odom_in_map(self):
        # 自分の位置を合わせる部品(AMCL など)がなければ、地図とオドメトリは同じとみなす
        self.state.set_odom_in_map(*(self.lookup(self.map_frame, self.odom_frame)
                                     or (0.0, 0.0, 0.0)))

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


class ConsoleWindow(QMainWindow):
    """
    ウィンドウ(フィールドエディタと同じ並び)。キー入力を受け取り、30 回/秒で表示を更新する.

        上   … 道具バー: 停止 / 再開、状態、足回りの種類、軌跡を消す
        中央 … 地図(/map)の上に、ロボットの位置と向き、走った跡、LiDAR の点
        右   … タブ: 操縦(速さ + WASD)/ ロボット(各モジュールの向き)/ LiDAR / 状態 /
               通信(STM32 ごとの指令・状態の中身と回数)/ グラフ(指令と実際の値の時間変化)
        下   … 位置・向き・速さ
    """

    TITLE = 'ロボットコンソール'
    SIDE_WIDTH = 470            # 右の欄の幅(タブが全部見える幅)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.keys = set()           # 押しているキー('w' など。Shift は 'shift')
        self.stopped = False
        self.header_key = None      # 道具バーを作り直すかどうかの判定用
        self.graph_keys = []        # グラフで選べるもの [(題名, 単位, 値の取り出し方)]

        self.setWindowTitle(self.TITLE)
        self.setFocusPolicy(Qt.StrongFocus)
        self.resize(1300, 820)
        self.setMinimumSize(900, 700)

        self.robot = widgets.RobotView(node.max_linear, node.max_angular)
        self.speed = widgets.SpeedGauges(node.max_linear, node.max_angular)
        self.keypad = widgets.KeyPad(f'{node.linear_speed:g} m/s ・ Shift ×{node.turbo_scale:g}')
        self.keypad.keyChanged.connect(self.on_pad_key)
        self.trail = widgets.TrailView()
        self.lidar = widgets.LidarView()
        self.status = widgets.StatusList()
        self.status.setMinimumHeight(140)
        self.status.setMaximumHeight(16777215)          # タブの中では、高さいっぱいに使う
        self.stm_table = widgets.StmTable()
        self.plot = widgets.TimePlot()
        self.graph_choice = QComboBox()
        self.graph_choice.setFocusPolicy(Qt.NoFocus)

        self.make_toolbar()
        self.make_body()

        self.view_timer = QTimer(self)
        self.view_timer.timeout.connect(self.refresh)
        self.view_timer.start(33)
        self.key_timer = QTimer(self)
        self.key_timer.timeout.connect(self.send_keys)
        self.key_timer.start(int(1000 / node.rate_hz))
        self.refresh()

    def make_toolbar(self):
        bar = QToolBar('操作')
        bar.setMovable(False)
        bar.setFocusPolicy(Qt.NoFocus)
        self.addToolBar(bar)
        self.stop_button = QPushButton('')
        self.stop_button.setFocusPolicy(Qt.NoFocus)     # スペースキーなどで勝手に押されないように
        self.stop_button.setCursor(Qt.PointingHandCursor)
        self.stop_button.setMinimumSize(170, 34)
        self.stop_button.clicked.connect(lambda: self.node.publish_stop(not self.stopped))
        bar.addWidget(self.stop_button)
        bar.addSeparator()
        self.pill = QLabel('')
        bar.addWidget(QLabel(' 状態 '))
        bar.addWidget(self.pill)
        bar.addSeparator()
        self.drive_label = QLabel('')
        bar.addWidget(QLabel(' 足回り '))
        bar.addWidget(self.drive_label)
        bar.addSeparator()
        clear = QAction('軌跡を消す', self)
        clear.triggered.connect(self.node.state.clear_trail)
        bar.addAction(clear)
        bar.addSeparator()
        for text, attr in (('地図', 'show_map'), ('LiDAR の点', 'show_scan')):
            toggle = QAction(text, self)
            toggle.setCheckable(True)
            toggle.setChecked(True)
            toggle.toggled.connect(lambda on, a=attr: (setattr(self.trail, a, on),
                                                       self.trail.update()))
            bar.addAction(toggle)
        self.extra_toolbar(bar)
        for button in bar.findChildren(QToolButton):
            button.setFocusPolicy(Qt.NoFocus)

    def extra_toolbar(self, bar):
        """シミュレーターなどが、道具バーに足すところ."""

    def extra_tabs(self, tabs):
        """シミュレーターなどが、右のタブを足すところ."""

    def on_refresh(self, snap):
        """シミュレーターなどが、表示を更新するところ(30 回/秒)."""

    def make_graph_tab(self):
        tab = QWidget()
        column = QVBoxLayout(tab)
        row = QHBoxLayout()
        row.addWidget(QLabel('表示するもの'))
        row.addWidget(self.graph_choice, 1)
        column.addLayout(row)
        column.addWidget(self.plot, 1)
        note = QLabel('点線が指令、実線が実際の値。最近 20 秒分。'
                      'stm<ID> は STM32 との通信の値(スワーブなら 1 が向き [rad]、2 がタイヤの速さ [rad/s])')
        note.setWordWrap(True)
        column.addWidget(note)
        return tab

    def update_graph_choices(self, snap):
        keys = [('速さ: 前後', 'm/s', ('body', 0)), ('速さ: 左右', 'm/s', ('body', 1)),
                ('速さ: 回転', 'rad/s', ('body', 2))]
        for row in snap.stm_rows:
            for i in (0, 1):
                keys.append((f'stm{row.dev_id}: 値 {i + 1}', '', ('stm', row.dev_id, i)))
        if [k[0] for k in keys] != [k[0] for k in self.graph_keys]:
            current = self.graph_choice.currentText()
            self.graph_keys = keys
            self.graph_choice.blockSignals(True)
            self.graph_choice.clear()
            self.graph_choice.addItems([k[0] for k in keys])
            index = self.graph_choice.findText(current)
            self.graph_choice.setCurrentIndex(max(0, index))
            self.graph_choice.blockSignals(False)

    def update_graph(self, snap):
        self.update_graph_choices(snap)
        index = self.graph_choice.currentIndex()
        if not (0 <= index < len(self.graph_keys)):
            return
        title, unit, key = self.graph_keys[index]
        points = []
        for sample in snap.history:
            if key[0] == 'body':
                actual = sample.actual[key[1]] if sample.actual is not None else None
                points.append((sample.t, sample.command[key[1]], actual))
            else:
                command, state = sample.stm.get(key[1], ((), ()))
                points.append((sample.t, command[key[2]] if len(command) > key[2] else None,
                               state[key[2]] if len(state) > key[2] else None))
        self.plot.set_series(title, unit, points)

    def make_body(self):
        tabs = QTabWidget()
        tabs.setFocusPolicy(Qt.NoFocus)
        tabs.tabBar().setFocusPolicy(Qt.NoFocus)
        tabs.setMinimumWidth(self.SIDE_WIDTH)
        drive = QWidget()
        column = QVBoxLayout(drive)
        self.speed.setMinimumHeight(210)                # 3 本のバーが切れない高さ
        for title, body, stretch in (('速さ(白い印 = 指令、バー = 実際)', self.speed, 0),
                                     ('操縦(クリックでも押せる)', self.keypad, 1)):
            group = QGroupBox(title)
            inner = QVBoxLayout(group)
            inner.addWidget(body)
            column.addWidget(group, stretch)
        tabs.addTab(drive, '操縦')
        tabs.addTab(self.robot, 'ロボット')
        tabs.addTab(self.lidar, 'LiDAR')
        tabs.addTab(self.status, '状態')
        tabs.addTab(self.stm_table, '通信')
        tabs.addTab(self.make_graph_tab(), 'グラフ')
        self.extra_tabs(tabs)
        self.tabs = tabs

        help_text = QLabel(
            '<b>操縦</b>  W / S 前後、A / D 左右、Q / E 回転、Shift で速く(このウィンドウを選んでいる間)<br>'
            '<b>停止</b>  Esc かスペース。「再開」を押すまで止まったまま<br>'
            '<span style="color:#c0392b">停止ボタンはソフトウェアの停止です。本物の非常停止スイッチ'
            '(電源を切るもの)は、必ず別に付けてください</span>')
        help_text.setWordWrap(True)
        side = QWidget()
        side_layout = QVBoxLayout(side)
        side_layout.setContentsMargins(0, 0, 0, 0)
        side_layout.addWidget(tabs, 1)
        side_layout.addWidget(help_text)

        body = QWidget()
        row = QHBoxLayout(body)
        row.setContentsMargins(4, 4, 4, 4)
        row.addWidget(self.trail, 1)
        row.addWidget(side)
        self.setCentralWidget(body)

    # ---- 表示 ----

    def refresh(self):
        if not rclpy.ok():
            self.close()
            return
        now = time.monotonic()
        self.node.state.sample(now)                     # グラフ用に記録する
        snap = self.node.state.snapshot(now)
        self.stopped = snap.stopped
        for view in (self.robot, self.speed, self.trail, self.lidar, self.status,
                     self.stm_table):
            view.set_snapshot(snap)
        if self.tabs.currentWidget() is self.plot.parentWidget():
            self.update_graph(snap)
        self.on_refresh(snap)
        self.keypad.set_keys(self.keys, self.isActiveWindow(), snap.stopped)
        self.update_header(snap)
        x, y, yaw = snap.map_pose
        speed = f'{math.hypot(snap.actual[0], snap.actual[1]):.2f} m/s' \
            if snap.actual is not None else '--'
        self.statusBar().showMessage(
            f'x {x:+.2f} m    y {y:+.2f} m    向き {math.degrees(yaw):+.0f}°    速さ {speed}')

    def update_header(self, snap):
        key = (snap.level, snap.summary, snap.stopped, snap.drive)
        if key == self.header_key:      # 変わったときだけ作り直す
            return
        self.header_key = key
        hex_color = th.LEVEL_COLOR[snap.level]
        self.pill.setText(f'●  {snap.summary}')
        self.pill.setStyleSheet(f'color: {hex_color}; font-weight: bold;')
        names = {'swerve': 'スワーブ', 'omni': 'オムニ'}
        self.drive_label.setText(names.get(snap.drive, '(データ待ち)'))
        if snap.stopped:
            base, hover, text, label = th.GOOD, '#22c55e', '#ffffff', '▶  再開'
        else:
            base, hover, text, label = th.BAD, '#ef4444', '#ffffff', '■  停止(Esc)'
        self.stop_button.setText(label)
        self.stop_button.setStyleSheet(
            f'QPushButton {{ background: {base}; color: {text}; border: none; border-radius: 6px;'
            f' font-weight: bold; padding: 4px 18px; }}'
            f' QPushButton:hover {{ background: {hover}; }}')

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


def main(node_class=None, window_class=None):
    rclpy.init()
    app = QApplication(sys.argv[:1])
    # 画面の処理の中でエラーが起きても、アプリごと落ちないようにする(内容は表示する)
    sys.excepthook = lambda *error: traceback.print_exception(*error)
    node = (node_class or ConsoleNode)()
    window = (window_class or ConsoleWindow)(node)
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
