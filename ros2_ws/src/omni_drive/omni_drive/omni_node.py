"""
オムニホイールの足回りを動かすノード.

    /cmd_vel (geometry_msgs/Twist)      … ロボット全体の目標速度。キーボード・ゲームパッド・Nav2 で同じ
          │                               linear.x 前 [m/s]、linear.y 左 [m/s]、angular.z 回転 [rad/s]
          ▼
    【このノード】逆運動学(kinematics.py)
          │
          ▼
    stm<ID>/command (Float32MultiArray) … [モーター1 の目標速度, モーター2 の目標速度] [rad/s]
          │  STM32 1 枚でホイール 2 個(モーター 2 個の速度制御。ファームウェアは APP なしの通常版)
          │  stm_bridge が CAN / USB シリアルで送る(sim のときは fake_wheels)
          ▼
    stm<ID>/state   (Float32MultiArray) … [モーター1 の実際の速度, モーター2 の実際の速度]
          │
          ▼
    【このノード】順運動学 → /odom と TF(odom → base_link)、/omni/markers(rviz2 の矢印)

どのホイールがどの基板のどのモーターかは、config/omni.yaml の device_ids と channels で決める。
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from swerve_drive import robot_status
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .kinematics import integrate_pose, OmniKinematics, Wheel


class OmniNode(Node):

    def __init__(self):
        super().__init__('omni_drive')
        p = self.declare_parameter
        names = list(p('wheel_names', ['front_left', 'front_right', 'rear_left',
                                       'rear_right']).value)
        xs = list(p('wheel_x', [0.25, 0.25, -0.25, -0.25]).value)
        ys = list(p('wheel_y', [0.25, -0.25, 0.25, -0.25]).value)
        dirs = list(p('wheel_direction_deg', [135.0, 45.0, -135.0, -45.0]).value)
        ids = list(p('device_ids', [1, 1, 2, 2]).value)
        channels = list(p('channels', [1, 2, 1, 2]).value)
        signs = list(p('wheel_sign', [1, 1, 1, 1]).value)
        if not len(names) == len(xs) == len(ys) == len(dirs) == len(ids) == len(channels) \
                == len(signs):
            raise ValueError('wheel_names / wheel_x / wheel_y / wheel_direction_deg / '
                             'device_ids / channels / wheel_sign の数をそろえてください')
        if any(c not in (1, 2) for c in channels):
            raise ValueError('channels は 1(モーター1)か 2(モーター2)にしてください')
        if len(set(zip(ids, channels))) != len(ids):
            raise ValueError('同じ基板の同じモーターに、2 個のホイールが割り当てられています')
        self.kinematics = OmniKinematics(
            [Wheel(n, x, y, math.radians(d)) for n, x, y, d in zip(names, xs, ys, dirs)],
            wheel_radius=p('wheel_radius', 0.05).value,
            max_wheel_speed=p('max_wheel_speed', 25.0).value)
        self.wheel_ids = ids
        self.wheel_slots = [c - 1 for c in channels]    # 0 = モーター1、1 = モーター2
        self.wheel_signs = [1.0 if s >= 0 else -1.0 for s in signs]
        self.cmd_timeout_s = p('cmd_timeout_s', 0.5).value
        self.state_timeout_s = p('state_timeout_s', 0.5).value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_link').value
        self.publish_tf = p('publish_tf', True).value
        rate_hz = p('rate_hz', 50.0).value

        self.boards = sorted(set(ids))
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None
        self.states = dict.fromkeys(self.boards)    # 基板ごとの (受け取った時刻, [速度 2 個])
        self.pose = [0.0, 0.0, 0.0]
        self.last_update = time.monotonic()

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.stopped = False        # 停止ボタン(/drive_stop)が押されている間は true
        self.create_subscription(Bool, robot_status.STOP_TOPIC, self.on_stop,
                                 robot_status.STOP_QOS)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self.publish_diagnostics)
        self.cmd_pubs = {}
        for board in self.boards:
            self.cmd_pubs[board] = self.create_publisher(
                Float32MultiArray, f'stm{board}/command', 10)
            self.create_subscription(
                Float32MultiArray, f'stm{board}/state',
                lambda msg, b=board: self.on_state(b, msg), 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'omni/markers', 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_timer(1.0 / rate_hz, self.update)
        self.create_timer(5.0, self.check_states)
        layout = ', '.join(f'{n}=stm{i}/{c}' for n, i, c in zip(names, ids, channels))
        self.get_logger().info(f'ホイール {layout}、タイヤ半径 {self.kinematics.wheel_radius} m')

    # ---- 受け取る ----

    def on_cmd_vel(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.get_logger().error(f'cmd_vel に NaN か inf が入っているので無視します: {values}',
                                    throttle_duration_sec=1.0)
            return
        self.cmd = values
        self.cmd_time = time.monotonic()

    def on_state(self, board, msg):
        if len(msg.data) < 2 or not all(math.isfinite(v) for v in msg.data[:2]):
            self.get_logger().warn(f'stm{board}/state を無視します: {list(msg.data)}',
                                   throttle_duration_sec=1.0)
            return
        self.states[board] = (time.monotonic(), [float(msg.data[0]), float(msg.data[1])])

    def wheel_speeds(self, now):
        """全部の基板から最新の状態が届いていれば、ホイールごとの速さを返す。なければ None."""
        if any(s is None or now - s[0] > self.state_timeout_s for s in self.states.values()):
            return None
        return [self.states[b][1][slot] * sign
                for b, slot, sign in zip(self.wheel_ids, self.wheel_slots, self.wheel_signs)]

    def on_stop(self, msg):
        if msg.data != self.stopped:
            self.get_logger().warn('停止ボタンが押されました。止まります' if msg.data
                                   else '停止ボタンが解除されました')
        self.stopped = msg.data

    def publish_diagnostics(self):
        """指令とSTM32の状態を /diagnostics に出す(1 秒ごと)."""
        now = time.monotonic()
        st = robot_status
        if self.stopped:
            cmd = st.make_status('omni_drive: 指令', st.WARN, '停止ボタンで止めています')
        elif self.cmd_time is not None and now - self.cmd_time <= self.cmd_timeout_s:
            vx, vy, omega = self.cmd
            cmd = st.make_status('omni_drive: 指令', st.OK, '/cmd_vel が届いています',
                                 前後=f'{vx:+.2f} m/s', 左右=f'{vy:+.2f} m/s',
                                 回転=f'{omega:+.2f} rad/s')
        else:
            cmd = st.make_status('omni_drive: 指令', st.OK, '指令なし(止まっています)')
        late = self.late_devices(now)
        if late:
            parts = st.make_status('omni_drive: STM32', st.ERROR,
                                   f'{", ".join(late)} の状態が届いていません')
        else:
            parts = st.make_status('omni_drive: STM32', st.OK, '全部の状態が届いています')
        msg = DiagnosticArray(status=[cmd, parts])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(msg)

    # ---- 周期処理 ----

    def update(self):
        now = time.monotonic()
        dt = now - self.last_update
        self.last_update = now

        # 1. 指令: cmd_vel が途切れたら止まる
        if self.stopped or self.cmd_time is None or now - self.cmd_time > self.cmd_timeout_s:
            vx = vy = omega = 0.0
        else:
            vx, vy, omega = self.cmd
        targets = self.kinematics.inverse(vx, vy, omega)
        per_board = {b: [0.0, 0.0] for b in self.boards}    # 使っていないモーターは 0
        for board, slot, sign, speed in zip(self.wheel_ids, self.wheel_slots, self.wheel_signs,
                                            targets):
            per_board[board][slot] = speed * sign
        for board, values in per_board.items():
            self.cmd_pubs[board].publish(Float32MultiArray(data=values))

        # 2. オドメトリ: 全部の基板の状態がそろっているときだけ進める
        measured = self.wheel_speeds(now)
        if measured is not None:
            body = self.kinematics.forward(measured)
            self.pose = list(integrate_pose(*self.pose, *body, dt))
            self.publish_odometry(body)
        self.publish_markers(targets, measured)

    def late_devices(self, now):
        return [f'stm{b}' for b, s in self.states.items()
                if s is None or now - s[0] > self.state_timeout_s]

    def check_states(self):
        late = self.late_devices(time.monotonic())
        if late:
            self.get_logger().warn(
                f'{", ".join(late)} から状態が届いていません(オドメトリは止まっています)。'
                'stm_bridge か fake_wheels が起動しているか確認してください')

    # ---- 送る ----

    def publish_odometry(self, body):
        x, y, yaw = self.pose
        stamp = self.get_clock().now().to_msg()
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = body
        self.odom_pub.publish(odom)

        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf.sendTransform(t)

    def publish_markers(self, targets, measured):
        """各ホイールの位置に、目標(赤)と実際(緑)の、床を押す向きと速さの矢印を出す."""
        markers = MarkerArray()
        for i, (wheel, target) in enumerate(zip(self.kinematics.wheels, targets)):
            markers.markers.append(self.arrow(i * 2, wheel, target, (1.0, 0.2, 0.2, 0.6)))
            if measured is not None:
                markers.markers.append(self.arrow(i * 2 + 1, wheel, measured[i],
                                                  (0.2, 1.0, 0.2, 1.0)))
        self.marker_pub.publish(markers)

    def arrow(self, marker_id, wheel, speed, rgba):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.ns = 'wheels'
        m.id = marker_id
        m.type = Marker.ARROW
        m.pose.position.x = wheel.x
        m.pose.position.y = wheel.y
        heading = wheel.direction if speed >= 0.0 else wheel.direction + math.pi
        m.pose.orientation.z = math.sin(heading / 2.0)
        m.pose.orientation.w = math.cos(heading / 2.0)
        m.scale.x = 0.02 + 0.2 * min(1.0, abs(speed) / self.kinematics.max_wheel_speed)
        m.scale.y = m.scale.z = 0.03
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.lifetime.nanosec = 500_000_000    # 届かなくなったら 0.5 秒で消す
        return m


def main():
    rclpy.init()
    node = OmniNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
