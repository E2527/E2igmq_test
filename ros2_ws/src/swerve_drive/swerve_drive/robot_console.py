"""
ロボットコンソール: 操縦・速さ・状態・停止を 1 つにまとめたウィンドウ(PC で動く).

    ┌ [■ 停止] 状態 ● すべて OK │ 足回り スワーブ │ 軌跡を消す ──────────┐
    │                                     │ [操縦] [ロボット] [状態]  │
    │          軌跡(地図)               │  速さ(指令と実際)        │
    │   ロボットの位置・向き・走った跡    │  WASD のキー              │
    │                                     │                          │
    ├ x +0.00 m  y +0.00 m  向き +0°  速さ 0.00 m/s ─────────────────┤
    (フィールドエディタ・設定画面と同じ、道具バー + 中央の図 + 右の欄 の並び)

    操縦 … W / S 前後、A / D 左右、Q / E 回転、Shift で速く。押している間だけ cmd_vel_teleop を出す
           (足回りの cmd_vel_mux が、自動操縦より優先して /cmd_vel にまとめる)。
           ほかのウィンドウに移ると、押していないことにして止まる
    停止 … 停止ボタン、Esc、スペースキー。/drive_stop に true を出し、足回りは /cmd_vel を無視して止まる。
           「再開」を押すまで止まったまま。自動操縦の途中なら、自動操縦もやめる
    自動操縦 … 「自動操縦」タブで地図をクリックすると、Nav2 にゴールを送る(navigate_to_pose)。
           探した道(/plan)を地図に表示する。手で操縦するキーを押すと、自動操縦をやめる

停止ボタンはソフトウェアの停止。**本物の非常停止スイッチ(モーターの電源を切るもの)は、必ず別に付けること。**

見るだけのトピック: /cmd_vel、/odom、swerve/markers、omni/markers、/diagnostics、
                   /map(地図)、/scan(LiDAR)、TF(map → odom、base_link → LiDAR)、
                   /plan(自動操縦の道)、cmd_vel_mux/active(どの指令を通しているか)
出すもの: cmd_vel_teleop、/drive_stop、/initialpose(「位置を合わせる」)、navigate_to_pose のゴール
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

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtWidgets import (QAction, QApplication, QButtonGroup, QComboBox, QGroupBox,
                             QHBoxLayout, QLabel, QMainWindow, QPushButton, QRadioButton,
                             QTabWidget, QToolBar, QToolButton, QVBoxLayout, QWidget)
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from . import console_state as cs
from . import console_theme as th
from . import console_tuning
from . import console_widgets as widgets
from . import robot_status
from .teleop_keys import KEY_DIRECTIONS, twist_from_keys

QT_KEYS = {Qt.Key_W: 'w', Qt.Key_A: 'a', Qt.Key_S: 's', Qt.Key_D: 'd', Qt.Key_Q: 'q',
           Qt.Key_E: 'e', Qt.Key_Shift: 'shift'}


STM_TOPIC = re.compile(r'^/stm(\d+)/(command|state)$')
MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def nav_robot_radius(default=0.35):
    """自動操縦の設定(robot_nav の config/nav2_params.yaml)の、ロボットの半径 [m]."""
    try:
        import os

        from ament_index_python.packages import get_package_share_directory
        import yaml
        path = os.path.join(get_package_share_directory('robot_nav'), 'config',
                            'nav2_params.yaml')
        with open(path, encoding='utf-8') as f:
            doc = yaml.safe_load(f)
        return float(doc['global_costmap']['global_costmap']['ros__parameters']['robot_radius'])
    except Exception:       # robot_nav がない(Pi など)、書き方がちがう
        return default


def quaternion_z_w(yaw):
    """水平面の向き [rad] → クォータニオンの (z, w)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


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

        # 手で操縦する指令。足回りの cmd_vel_mux が、自動操縦(cmd_vel_nav)より優先して /cmd_vel に出す
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
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
        # ゲインの読み書き(「調整」タブ): stm<ID>/param に頼み、stm<ID>/param_reply で返事を受け取る
        self.stm_subscribed = set()
        self.param_pubs = {}
        self.create_timer(1.0, self.discover_stm)
        # 自動操縦(Nav2)。起動していなければ、ゴールを送れないだけ
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.through_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.robot_radius = nav_robot_radius()  # ゴールが障害物に近すぎないかを見るのに使う
        self.goal_handle = None
        self.goal_serial = 0            # 送ったゴールの番号(古いゴールの知らせを見分ける)
        self.dropped_serial = None      # 受け付けの返事を待っている間にやめたゴールの番号
        self.next_stages = []           # 今のゴールに着いたあとに送るゴール(着いてから回るとき)
        # Nav2 の、道に沿って走る部品を選ぶ(標準のビヘイビアツリーの ControllerSelector が読む)
        self.controller_pub = self.create_publisher(String, 'controller_selector', MAP_QOS)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_subscription(String, 'cmd_vel_mux/active',
                                 lambda msg: self.state.on_mux(msg.data, time.monotonic()), 10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, MAP_QOS)
        self.create_timer(1.0, lambda: self.state.set_nav_ready(self.nav_client.server_is_ready()))

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
            if dev_id not in self.param_pubs:
                self.param_pubs[dev_id] = self.create_publisher(
                    Float32MultiArray, f'/stm{dev_id}/param', 10)
                self.create_subscription(
                    Float32MultiArray, f'/stm{dev_id}/param_reply',
                    lambda msg, d=dev_id: self.on_param_reply(d, msg), 10)

    def on_amcl(self, msg):
        cov = msg.pose.covariance
        spread_m = math.sqrt(max(0.0, (cov[0] + cov[7]) / 2.0))
        self.state.on_amcl(spread_m, math.sqrt(max(0.0, cov[35])), time.monotonic())

    def on_plan(self, msg):
        # 道は地図の座標(Nav2 の global_frame)で届く
        self.state.on_plan([(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses])

    def on_param_reply(self, dev_id, msg):
        if len(msg.data) >= 3:
            self.state.on_param_reply(dev_id, int(msg.data[0]), int(msg.data[1]), msg.data[2],
                                      time.monotonic())

    def request_param(self, dev_id, param_id, value=None):
        """STM32 のゲインを読む(value が None)/ 書く。返事は on_param_reply に届く."""
        pub = self.param_pubs.get(dev_id)
        if pub is not None:
            data = [float(param_id)] if value is None else [float(param_id), float(value)]
            pub.publish(Float32MultiArray(data=data))

    def update_odom_in_map(self):
        # 自分の位置を合わせる部品(AMCL など)がなければ、地図とオドメトリは同じとみなす
        pose = self.lookup(self.map_frame, self.odom_frame)
        if pose is None:
            self.state.set_odom_in_map(0.0, 0.0, 0.0)
        else:
            self.state.set_odom_in_map(*pose, time.monotonic())

    # ---- 出す(画面のスレッド) ----

    def publish_keys(self, keys, turbo):
        """押しているキーから cmd_vel_teleop を出す。離したときは 1 回だけ 0 を出して止める."""
        twist = twist_from_keys(keys, self.linear_speed, self.angular_speed, turbo,
                                self.turbo_scale)
        moving = any(v != 0.0 for v in twist)
        if moving and not self.was_moving and \
                self.state.snapshot(time.monotonic(), with_history=False).navigating:
            self.cancel_goal()          # 手で動かし始めたら、自動操縦はやめる
        if moving or self.was_moving:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = twist
            self.cmd_pub.publish(msg)
        self.was_moving = moving

    def publish_stop(self, stop):
        self.state.set_stop(stop, time.monotonic())
        self.stop_pub.publish(Bool(data=stop))
        if stop:
            self.cancel_goal()

    def pose_stamped(self, x, y, yaw):
        msg = NavigateToPose.Goal().pose
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = quaternion_z_w(yaw)
        return msg

    def publish_initial_pose(self, x, y, yaw):
        """AMCL に「ロボットは今ここにいる」と知らせる(コンソールの「位置を合わせる」)."""
        msg = PoseWithCovarianceStamped()
        stamped = self.pose_stamped(x, y, yaw)
        msg.header, msg.pose.pose = stamped.header, stamped.pose
        # ばらつき: 位置 25 cm、向き 15°くらい(クリックで決めるので大きめ)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25 ** 2
        msg.pose.covariance[35] = math.radians(15.0) ** 2
        self.initial_pub.publish(msg)

    # ---- 自動操縦(ゴールを送るのは画面のスレッド、知らせは ROS のスレッド) ----

    def send_goal(self, x, y, yaw, turn=cs.TURN_WHILE_MOVING):
        """Nav2 にゴールを 1 つ送る(send_route の、点が 1 つのとき)."""
        return self.send_route([(x, y, yaw)], turn)

    def check_route(self, points, snap):
        """経由地点の並びを送れるか。送れなければ理由を返す."""
        client = self.nav_client if len(points) == 1 else self.through_client
        if not client.server_is_ready():
            return '自動操縦(Nav2)が起動していません'
        if snap.stopped:
            return '停止中です。「再開」を押してから送ってください'
        # ロボットの半径より障害物に近い点には、ぶつからずに行けない(着く前にうろうろする)
        for i, (x, y, _yaw) in enumerate(points):
            gap = snap.grid.clearance(x, y, self.robot_radius) if snap.grid is not None else 1e9
            if gap < self.robot_radius:
                name = 'ゴール' if i == len(points) - 1 else f'{i + 1} 番目の点'
                return (f'{name}が壁や障害物に近すぎます({gap * 100:.0f} cm)。ロボットの半径 '
                        f'{self.robot_radius * 100:.0f} cm より離して置いてください(地図の点線の円)')
        return None

    def send_route(self, points, turn=cs.TURN_WHILE_MOVING):
        """
        経由地点の並び [(x, y, 向き)](最後がゴール)を Nav2 に送る。走っている途中なら、新しい道に変わる.

        turn … 向きをいつ変えるか(cs.TURN_WHILE_MOVING / cs.TURN_ON_ARRIVAL)。送れたら True
        """
        if not points:
            return False
        snap = self.state.snapshot(time.monotonic(), with_history=False)
        problem = self.check_route(points, snap)
        if problem:
            if snap.navigating:     # 走っている途中なら、今の道のまま知らせるだけ
                self.state.set_nav(snap.nav_phase, problem, snap.nav_remaining)
            else:
                self.state.set_nav(cs.NAV_FAILED, problem)
            return False
        self.state.set_nav(cs.NAV_SENDING, cs.TURN_TEXT.get(turn, ''), goal=list(points))
        stages = cs.goal_stages(turn, points, snap.map_pose[2])
        self.send_stage(stages[0], stages[1:])
        return True

    def send_stage(self, stage, rest):
        """
        ゴールを送る(点が 1 つなら navigate_to_pose、2 つ以上なら navigate_through_poses).

        rest は、着いたあとに続けて送るもの(着いてから回るとき)。
        """
        controller, poses = stage
        self.controller_pub.publish(String(data=controller))
        if len(poses) == 1:
            client, goal = self.nav_client, NavigateToPose.Goal()
            goal.pose = self.pose_stamped(*poses[0])
        else:
            client, goal = self.through_client, NavigateThroughPoses.Goal()
            goal.poses = [self.pose_stamped(*p) for p in poses]
        self.goal_serial += 1
        serial = self.goal_serial
        self.next_stages = list(rest)
        future = client.send_goal_async(
            goal, feedback_callback=lambda fb, n=serial: self.on_feedback(n, fb))
        future.add_done_callback(lambda f, n=serial: self.on_goal_response(n, f))

    def on_goal_response(self, serial, future):
        handle = future.result()
        if serial != self.goal_serial:          # もっと新しいゴールを送ったか、やめたあと
            if serial == self.dropped_serial and handle is not None and handle.accepted:
                handle.cancel_goal_async()      # やめたゴールが受け付けられていたら、やめさせる
            return
        if handle is None or not handle.accepted:
            self.state.set_nav(cs.NAV_FAILED, 'Nav2 がゴールを受け付けませんでした')
            return
        self.goal_handle = handle
        snap = self.state.snapshot(time.monotonic(), with_history=False)
        self.state.set_nav(cs.NAV_RUNNING, snap.nav_text)
        handle.get_result_async().add_done_callback(lambda f: self.on_result(serial, f))

    def on_feedback(self, serial, feedback):
        if serial == self.goal_serial:
            fb = feedback.feedback
            self.state.on_nav_feedback(fb.distance_remaining,
                                       getattr(fb, 'number_of_poses_remaining', None))

    def on_result(self, serial, future):
        if serial != self.goal_serial:
            return
        self.goal_handle = None
        response = future.result()
        status = response.status if response is not None else GoalStatus.STATUS_UNKNOWN
        if status == GoalStatus.STATUS_SUCCEEDED and self.next_stages:
            # 着いてから回る: 位置に着いたので、その場でゴールの向きに回る
            self.state.set_nav(cs.NAV_SENDING, '着きました。その場で向きを変えています')
            self.send_stage(self.next_stages[0], self.next_stages[1:])
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self.state.set_nav(cs.NAV_SUCCEEDED)
        elif status == GoalStatus.STATUS_CANCELED:
            self.state.set_nav(cs.NAV_CANCELED)
        else:
            result = response.result if response is not None else None
            code = getattr(result, 'error_code', 0)
            text = getattr(result, 'error_msg', '') or (f'エラーの番号 {code}' if code else '')
            self.state.set_nav(cs.NAV_FAILED, text or '道が見つからないか、行き詰まりました')

    def cancel_goal(self):
        """自動操縦をやめる(走っていなければ何もしない)."""
        handle = self.goal_handle
        self.next_stages = []
        if handle is None:
            if self.state.snapshot(time.monotonic(), with_history=False).navigating:
                self.dropped_serial = self.goal_serial  # 返事を待っている途中のゴールは捨てる
                self.goal_serial += 1
                self.state.set_nav(cs.NAV_CANCELED)
            return
        self.state.set_nav(cs.NAV_CANCELING)
        handle.cancel_goal_async()


class ConsoleWindow(QMainWindow):
    """
    ウィンドウ(フィールドエディタと同じ並び)。キー入力を受け取り、30 回/秒で表示を更新する.

        上   … 道具バー: 停止 / 再開、状態、足回りの種類、軌跡を消す
        中央 … 地図(/map)の上に、ロボットの位置と向き、走った跡、LiDAR の点
        右   … タブ: 操縦(速さ + WASD)/ 自動操縦(ゴールを置く)/ ロボット(各モジュールの向き)/
               LiDAR / 状態 / 通信(STM32 ごとの指令・状態の中身と回数)/
               グラフ(指令と実際の値の時間変化)/ 調整(STM32 のゲインを書き込み直さずに変える)
        下   … 位置・向き・速さ
    """

    TITLE = 'ロボットコンソール'
    SIDE_WIDTH = 500            # 右の欄の幅(タブが全部見える幅)

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
        self.nav_key = None         # 自動操縦のタブを書き直すかどうかの判定用
        self.waypoints = []         # 地図で並べた経由地点 [(x, y, 向き)](最後がゴール。Enter で走る)

        self.make_toolbar()
        self.make_body()
        self.trail.posePicked.connect(self.on_pose_picked)
        self.trail.robot_radius = getattr(node, 'robot_radius', None)
        self.update_points()

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

    def make_nav_tab(self):
        tab = QWidget()
        column = QVBoxLayout(tab)
        self.nav_label = QLabel('')
        self.nav_label.setWordWrap(True)
        self.nav_label.setStyleSheet('font-weight: bold;')
        column.addWidget(self.nav_label)
        self.nav_detail = QLabel('')
        self.nav_detail.setWordWrap(True)
        column.addWidget(self.nav_detail)

        goal = QGroupBox('経由地点とゴール(地図をクリックして並べ、Enter で走る)')
        goal_layout = QVBoxLayout(goal)
        self.points_label = QLabel('')
        self.points_label.setWordWrap(True)
        goal_layout.addWidget(self.points_label)
        buttons = QHBoxLayout()
        self.go_button = QPushButton('走る(Enter)')
        self.undo_button = QPushButton('1 つ消す(BackSpace)')
        self.clear_button = QPushButton('全部消す')
        for button, slot in ((self.go_button, self.start_route),
                             (self.undo_button, self.undo_point),
                             (self.clear_button, self.clear_points)):
            button.setFocusPolicy(Qt.NoFocus)
            button.clicked.connect(slot)
            buttons.addWidget(button)
        goal_layout.addLayout(buttons)
        # 向きをいつ変えるか
        turn_row = QHBoxLayout()
        turn_row.addWidget(QLabel('向きを変える:'))
        self.turn_group = QButtonGroup(self)
        self.turn_buttons = {}
        for mode in (cs.TURN_WHILE_MOVING, cs.TURN_ON_ARRIVAL):
            button = QRadioButton(cs.TURN_TEXT[mode])
            button.setFocusPolicy(Qt.NoFocus)
            self.turn_group.addButton(button)
            self.turn_buttons[mode] = button
            turn_row.addWidget(button)
        turn_row.addStretch(1)
        self.turn_buttons[cs.TURN_WHILE_MOVING].setChecked(True)
        goal_layout.addLayout(turn_row)
        cancel = QPushButton('自動操縦をやめる')
        cancel.setFocusPolicy(Qt.NoFocus)
        cancel.clicked.connect(self.node.cancel_goal)
        goal_layout.addWidget(cancel)
        column.addWidget(goal)

        place = QGroupBox('位置を合わせる(AMCL)')
        place_layout = QVBoxLayout(place)
        self.amcl_label = QLabel('')
        self.amcl_label.setWordWrap(True)
        place_layout.addWidget(self.amcl_label)
        initial = QPushButton('地図をクリックして、今いる位置を教える')
        initial.setFocusPolicy(Qt.NoFocus)
        initial.clicked.connect(lambda: self.start_pick(
            'initial', 'ロボットが今いる位置をクリックします(ドラッグすると、ロボットの向き)'))
        place_layout.addWidget(initial)
        hint = QLabel('AMCL は、LiDAR の点(赤)と地図の壁を重ねて、ずれを直します(動くたびと、'
                      '止まっている間は 3 秒ごと)。大きくずれて直らないときは、だいたいの位置を教えます。')
        hint.setWordWrap(True)
        place_layout.addWidget(hint)
        column.addWidget(place)

        note = QLabel('<b>このタブを開いている間は、地図をクリックするたびに経由地点を置けます</b>'
                      '(ドラッグで向き。向きを使うのはゴールだけ)。Enter で、置いた順に通って最後の点まで'
                      '走ります。走っている途中に並べて Enter を押すと、新しい道に変わります。'
                      '紫の線が自動操縦の探した道です。WASD などで手で操縦すると、自動操縦はやめます'
                      '(ゲームパッドで操縦している間は、そちらが優先されます)。'
                      '自動操縦は ./sim.sh nav か ./console.sh nav:=true で起動します。')
        note.setWordWrap(True)
        column.addWidget(note)
        column.addStretch(1)
        return tab

    def start_pick(self, mode, hint):
        self.trail.set_pick_mode(mode)
        self.statusBar().showMessage(hint, 5000)

    def on_pose_picked(self, mode, x, y, yaw):
        if mode == 'goal':          # 経由地点を 1 つ足す(最後の点がゴール。Enter で走る)
            self.waypoints.append((x, y, yaw))
            self.update_points()
        elif mode == 'initial':
            self.node.publish_initial_pose(x, y, yaw)
        self.setFocus()

    # ---- 経由地点 ----

    def update_points(self):
        self.trail.waypoints = list(self.waypoints)
        self.trail.update()
        n = len(self.waypoints)
        if n == 0:
            text = '地図をクリックすると、経由地点を置きます(ドラッグで向き)。最後に置いた点がゴールです'
        else:
            text = f'{n} か所(最後の {n} 番がゴール)。Enter で走ります'
        self.points_label.setText(text)
        self.go_button.setEnabled(n > 0)
        self.undo_button.setEnabled(n > 0)
        self.clear_button.setEnabled(n > 0)

    def start_route(self):
        """並べた経由地点を順番に通って、最後の点(ゴール)まで走る."""
        if not self.waypoints:
            return
        turn = next(m for m, b in self.turn_buttons.items() if b.isChecked())
        if self.node.send_route(list(self.waypoints), turn):
            self.waypoints = []         # 送ったら、次の道を並べられるように空にする
            self.update_points()
        self.setFocus()

    def undo_point(self):
        if self.waypoints:
            self.waypoints.pop()
            self.update_points()

    def clear_points(self):
        self.waypoints = []
        self.update_points()

    def update_amcl(self, snap):
        if snap.amcl_age is None or snap.amcl_age > 30.0:
            text = 'AMCL は動いていません(./sim.sh か ./console.sh amcl:=true で起動)'
        else:
            spread_m, spread_rad = snap.amcl_spread
            text = (f'最後に合わせたのは {snap.amcl_age:.0f} 秒前(ばらつき {spread_m * 100:.0f} cm、'
                    f'{math.degrees(spread_rad):.0f}°)')
            if snap.last_fix is not None:
                dist, dyaw, age = snap.last_fix
                text += (f'<br>最後に直した量: {dist * 100:.1f} cm、{math.degrees(dyaw):+.1f}°'
                         f'({age:.0f} 秒前)')
        if text != self.amcl_label.text():
            self.amcl_label.setText(text)

    def update_nav(self, snap):
        self.update_amcl(snap)
        key = (snap.nav_ready, snap.nav_phase, snap.nav_text,
               None if snap.nav_remaining is None else round(snap.nav_remaining, 1),
               snap.nav_points_left, snap.mux_source)
        if key == self.nav_key:         # 変わったときだけ書き直す
            return
        self.nav_key = key
        colors = {cs.NAV_SUCCEEDED: th.GOOD, cs.NAV_FAILED: th.BAD, cs.NAV_RUNNING: th.ACCENT_2}
        if not snap.nav_ready and not snap.navigating:
            text, color = '自動操縦(Nav2)が起動していません', th.DIM
        else:
            text, color = cs.NAV_TEXT[snap.nav_phase], colors.get(snap.nav_phase, th.TEXT)
            if snap.nav_phase == cs.NAV_RUNNING and snap.nav_remaining is not None:
                left = snap.nav_points_left
                text += f'(残り {snap.nav_remaining:.1f} m' + \
                    (f'、あと {left} か所)' if left and len(snap.route) > 1 else ')')
        self.nav_label.setText(text)
        self.nav_label.setStyleSheet(f'font-weight: bold; color: {color};')
        sources = {'teleop': '手で操縦', 'nav': '自動操縦', '': 'どれも来ていない'}
        lines = [snap.nav_text] if snap.nav_text else []
        if snap.mux_source is None:
            source = '分からない(cmd_vel_mux が動いていない)'
        else:
            source = sources.get(snap.mux_source, snap.mux_source)
        lines.append(f'足回りに通している指令: {source}')
        self.nav_detail.setText('\n'.join(lines))

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
        self.nav_tab = self.make_nav_tab()
        tabs.addTab(self.nav_tab, '自動操縦')
        tabs.addTab(self.robot, 'ロボット')
        tabs.addTab(self.lidar, 'LiDAR')
        tabs.addTab(self.status, '状態')
        tabs.addTab(self.stm_table, '通信')
        tabs.addTab(self.make_graph_tab(), 'グラフ')
        self.tune_tab = console_tuning.TuneTab(self.node)
        tabs.addTab(self.tune_tab, '調整')
        self.extra_tabs(tabs)
        self.tabs = tabs
        # 自動操縦のタブを開いている間は、地図をクリックするだけでゴールを置ける
        tabs.currentChanged.connect(lambda _: self.trail.set_click_mode(
            'goal' if tabs.currentWidget() is self.nav_tab else None))

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
        if self.tabs.currentWidget() is self.tune_tab:
            self.tune_tab.refresh(snap)
        self.update_nav(snap)
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
        elif event.key() in (Qt.Key_Return, Qt.Key_Enter) and self.waypoints:
            self.start_route()          # 並べた経由地点を走る
        elif event.key() == Qt.Key_Backspace and self.waypoints:
            self.undo_point()
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
