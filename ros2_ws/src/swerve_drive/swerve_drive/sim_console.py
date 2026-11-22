"""
シミュレーター: ロボットコンソールの全部の機能 + シミュレーションを操る機能(PC で、実機なしで使う).

    ./sim.sh                                  # ふつうはこれで開く
    ros2 run swerve_drive simulator           # 直接開く(シミュレーションのノードは別に起動しておく)

ロボットコンソール(robot_console.py)と同じ画面に、「シミュレーション」のタブを足したもの:
    ロボットを置く … 地図をクリック(ドラッグで向き)、またはフィールドのスタート位置のボタン
    障害物を置く   … 地図をクリック。シミュレーション用の LiDAR に映る(地図には入らない)
    わざと故障させる … STM32 を止める / 状態に NaN / 通信を落とす / タイヤの滑り / 向きのずれ /
                     LiDAR のばらつき
    ずれ           … 本当の位置(灰色の三角)と、オドメトリから見た位置のずれ

出すもの: /sim/teleport(置き直し)、/sim/obstacles(障害物)、/sim/faults(故障)
見るもの: /sim/truth(本当の位置)、/sim/start_poses(スタート位置)
シミュレーションの世界は field_map の sim_world_node / fake_lidar、STM32 のふりは fake_modules など。

実機を動かすときは、シミュレーターではなくロボットコンソール(./console.sh)を使う。
"""

import json
import math
import time

from geometry_msgs.msg import PoseStamped
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QCheckBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
                             QHBoxLayout, QLabel, QPushButton, QScrollArea, QVBoxLayout, QWidget)
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from . import console_theme as th
from . import robot_console as rc
from . import sim_faults as sf

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SimulatorNode(rc.ConsoleNode):
    """ロボットコンソールのノード + シミュレーションとのやり取り."""

    def __init__(self):
        super().__init__()
        self.start_poses = {}
        self.teleport_pub = self.create_publisher(PoseStamped, '/sim/teleport', 10)
        self.faults_pub = self.create_publisher(String, sf.FAULT_TOPIC, LATCHED)
        self.obstacles_pub = self.create_publisher(String, '/sim/obstacles', LATCHED)
        self.create_subscription(PoseStamped, '/sim/truth', self.on_truth, 10)
        self.create_subscription(String, '/sim/start_poses', self.on_start_poses, LATCHED)

    def on_truth(self, msg):
        p = msg.pose
        self.state.set_truth(p.position.x, p.position.y, rc.yaw_of(p.orientation),
                             time.monotonic())

    def on_start_poses(self, msg):
        try:
            data = json.loads(msg.data)
            self.start_poses = {str(k): (float(v[0]), float(v[1]), math.radians(float(v[2])))
                                for k, v in data.items()}
        except (ValueError, TypeError, AttributeError, IndexError):
            self.start_poses = {}

    def teleport(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.teleport_pub.publish(msg)

    def publish_faults(self, faults):
        self.faults_pub.publish(String(data=sf.dump_faults(faults)))

    def publish_obstacles(self, circles):
        self.obstacles_pub.publish(String(data=json.dumps([list(c) for c in circles])))


class SimulatorWindow(rc.ConsoleWindow):
    """ロボットコンソールの画面 + 「シミュレーション」のタブ."""

    TITLE = 'シミュレーター'
    SIDE_WIDTH = 640            # 「シミュレーション」のタブの分だけ広くする

    def __init__(self, node):
        self.obstacles = []
        self.fault_boxes = {}       # (ID, 'dead' / 'nan') → チェックボックス
        self.pose_buttons = {}      # スタート位置の名前 → ボタン
        super().__init__(node)
        self.trail.pointPicked.connect(self.on_point_picked)
        self.send_faults()          # 起動したときは故障なし(前に残っていた設定を消す)
        self.node.publish_obstacles([])

    # ---- 部品 ----

    def extra_toolbar(self, bar):
        bar.addSeparator()
        self.sim_badge = QLabel('  シミュレーション  ')
        self.sim_badge.setStyleSheet(f'background: {th.ACCENT_2}; color: white; font-weight: bold;'
                                     ' border-radius: 4px; padding: 3px 6px;')
        bar.addWidget(self.sim_badge)
        self.warn_label = QLabel('')
        self.warn_label.setStyleSheet(f'color: {th.BAD}; font-weight: bold;')
        bar.addWidget(self.warn_label)

    def extra_tabs(self, tabs):
        page = QWidget()
        column = QVBoxLayout(page)

        place = QGroupBox('ロボットを置く')
        place_layout = QVBoxLayout(place)
        pick = QPushButton('地図をクリックして置く(ドラッグで向き)')
        pick.clicked.connect(lambda: self.start_pick(
            'pose', '地図をクリックして置きます(ドラッグすると向き)'))
        place_layout.addWidget(pick)
        self.pose_row = QHBoxLayout()
        origin = QPushButton('原点 (0, 0)')
        origin.clicked.connect(lambda: self.node.teleport(0.0, 0.0, 0.0) or self.after_teleport())
        self.pose_row.addWidget(origin)
        place_layout.addLayout(self.pose_row)
        self.error_label = QLabel('ずれ: --')
        place_layout.addWidget(self.error_label)
        column.addWidget(place)

        obstacles = QGroupBox('障害物を置く(LiDAR に映る。地図には入らない)')
        obs_layout = QFormLayout(obstacles)
        self.obstacle_radius = self.spin(0.02, 2.0, 0.15, 0.05, ' m')
        obs_layout.addRow('半径', self.obstacle_radius)
        buttons = QHBoxLayout()
        add = QPushButton('地図をクリックして置く')
        add.clicked.connect(lambda: self.start_pick('point', '地図をクリックして障害物を置きます'))
        clear = QPushButton('全部消す')
        clear.clicked.connect(self.clear_obstacles)
        buttons.addWidget(add)
        buttons.addWidget(clear)
        obs_layout.addRow(buttons)
        column.addWidget(obstacles)

        faults = QGroupBox('わざと故障させる')
        fault_layout = QVBoxLayout(faults)
        self.fault_grid = QGridLayout()
        self.fault_grid.addWidget(QLabel('STM32'), 0, 0)
        self.fault_grid.addWidget(QLabel('止める'), 0, 1)
        self.fault_grid.addWidget(QLabel('状態に NaN'), 0, 2)
        fault_layout.addLayout(self.fault_grid)
        form = QFormLayout()
        self.drop = self.spin(0, 100, 0, 10, ' %', decimals=0)
        form.addRow('状態を落とす割合', self.drop)
        self.slip = self.spin(0, 90, 0, 5, ' %', decimals=0)
        form.addRow('タイヤの滑り', self.slip)
        self.drift = self.spin(-30, 30, 0, 0.5, ' °/秒', decimals=1)
        form.addRow('向きのずれ', self.drift)
        self.lidar_noise = self.spin(0, 0.5, 0.01, 0.01, ' m')
        form.addRow('LiDAR のばらつき', self.lidar_noise)
        fault_layout.addLayout(form)
        for box in (self.drop, self.slip, self.drift, self.lidar_noise):
            box.valueChanged.connect(self.send_faults)
        reset = QPushButton('故障を全部なくす')
        reset.clicked.connect(self.reset_faults)
        fault_layout.addWidget(reset)
        column.addWidget(faults)
        column.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        scroll.setFocusPolicy(Qt.NoFocus)
        tabs.addTab(scroll, 'シミュレーション')
        tabs.setCurrentWidget(scroll)

    @staticmethod
    def spin(low, high, value, step, suffix, decimals=2):
        box = QDoubleSpinBox()
        box.setRange(low, high)
        box.setDecimals(decimals)
        box.setSingleStep(step)
        box.setValue(value)
        box.setSuffix(suffix)
        box.setKeyboardTracking(False)
        box.setFocusPolicy(Qt.ClickFocus)
        return box

    # ---- ロボットを置く・障害物 ----

    def on_pose_picked(self, mode, x, y, yaw):
        if mode == 'pose':
            self.node.teleport(x, y, yaw)
            self.after_teleport()
        else:
            super().on_pose_picked(mode, x, y, yaw)

    def after_teleport(self):
        self.node.state.clear_trail()       # 前の場所からの軌跡は残さない
        self.setFocus()

    def on_point_picked(self, x, y):
        self.obstacles.append((round(x, 3), round(y, 3), round(self.obstacle_radius.value(), 3)))
        self.trail.obstacles = list(self.obstacles)
        self.node.publish_obstacles(self.obstacles)
        self.setFocus()

    def clear_obstacles(self):
        self.obstacles = []
        self.trail.obstacles = []
        self.node.publish_obstacles([])

    # ---- 故障 ----

    def current_faults(self):
        faults = dict(sf.DEFAULTS)
        faults['dead'] = frozenset(i for (i, kind), box in self.fault_boxes.items()
                                   if kind == 'dead' and box.isChecked())
        faults['nan'] = frozenset(i for (i, kind), box in self.fault_boxes.items()
                                  if kind == 'nan' and box.isChecked())
        faults['drop'] = self.drop.value() / 100.0
        faults['slip'] = self.slip.value() / 100.0
        faults['drift_deg_s'] = self.drift.value()
        faults['lidar_noise'] = self.lidar_noise.value()
        return faults

    def send_faults(self, *_):
        self.node.publish_faults(self.current_faults())

    def reset_faults(self):
        for box in self.fault_boxes.values():
            box.blockSignals(True)
            box.setChecked(False)
            box.blockSignals(False)
        for spin, value in ((self.drop, 0), (self.slip, 0), (self.drift, 0),
                            (self.lidar_noise, 0.01)):
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)
        self.send_faults()

    def update_fault_rows(self, snap):
        """見つかった STM32 ごとに、「止める」「NaN」のチェックボックスを並べる."""
        for row in snap.stm_rows:
            if (row.dev_id, 'dead') in self.fault_boxes:
                continue
            r = self.fault_grid.rowCount()
            self.fault_grid.addWidget(QLabel(f'stm{row.dev_id}'), r, 0)
            for c, kind in ((1, 'dead'), (2, 'nan')):
                box = QCheckBox()
                box.setFocusPolicy(Qt.NoFocus)
                box.toggled.connect(self.send_faults)
                self.fault_grid.addWidget(box, r, c)
                self.fault_boxes[(row.dev_id, kind)] = box

    def update_pose_buttons(self):
        for name, pose in sorted(self.node.start_poses.items()):
            if name in self.pose_buttons:
                continue
            button = QPushButton(f'スタート: {name}')
            button.setFocusPolicy(Qt.NoFocus)
            button.clicked.connect(lambda _=False, n=name: (
                self.node.teleport(*self.node.start_poses[n]), self.after_teleport()))
            self.pose_row.addWidget(button)
            self.pose_buttons[name] = button

    # ---- 表示 ----

    def on_refresh(self, snap):
        self.update_fault_rows(snap)
        self.update_pose_buttons()
        if snap.truth is not None:
            x, y, yaw = snap.map_pose
            tx, ty, tyaw = snap.truth
            dist = math.hypot(x - tx, y - ty)
            dyaw = math.degrees(math.atan2(math.sin(yaw - tyaw), math.cos(yaw - tyaw)))
            self.error_label.setText(f'ずれ(本当の位置 = 灰色の三角): {dist * 100:.1f} cm、'
                                     f'向き {dyaw:+.1f}°')
        else:
            self.error_label.setText('ずれ: 本当の位置(/sim/truth)が届いていません')
        # 実機の stm_bridge の知らせが来ていたら、シミュレーターを使っていいか注意する
        real = any(r.name.startswith('stm_bridge') for r in snap.rows)
        self.warn_label.setText('  ⚠ 実機の stm_bridge が動いています。実機はロボットコンソールで'
                                if real else '')


def main():
    rc.main(SimulatorNode, SimulatorWindow)


if __name__ == '__main__':
    main()
