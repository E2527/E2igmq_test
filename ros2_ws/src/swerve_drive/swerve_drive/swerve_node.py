"""
独立ステアリング(スワーブ)の足回りを動かすノード.

    /cmd_vel (geometry_msgs/Twist)      … ロボット全体の目標速度。コントローラーでも Nav2 でも同じ
          │                               linear.x 前 [m/s]、linear.y 左 [m/s]、angular.z 回転 [rad/s]
          ▼
    【このノード】逆運動学(kinematics.py)
          │
          ▼
    stm<ID>/command (Float32MultiArray) … [向き rad, タイヤの速さ rad/s](モジュールごと)
          │  stm_bridge が CAN / USB シリアルで STM32 に送る(sim のときは fake_modules)
          ▼
    stm<ID>/state   (Float32MultiArray) … [今の向き rad, 今のタイヤの速さ rad/s]
          │
          ▼
    【このノード】順運動学 → /odom (nav_msgs/Odometry) と TF(odom → base_link)
                              /swerve/markers(rviz2 で各モジュールの向きを矢印で表示)

STM32 が向きをどう測るか(ロータリーエンコーダ、ステッピングモーターなど)は、
このノードには関係ない。STM32 が上の [向き, タイヤの速さ] の約束を守れば、センサーを変えても
ここは書き換えなくてよい。
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from . import robot_status
from .kinematics import cosine_scale, integrate_pose, Module, SwerveKinematics


class SwerveNode(Node):

    def __init__(self):
        super().__init__('swerve_drive')
        p = self.declare_parameter
        names = list(p('module_names', ['front_left', 'front_right', 'rear_left',
                                        'rear_right']).value)
        ids = list(p('device_ids', [1, 2, 3, 4]).value)
        xs = list(p('module_x', [0.25, 0.25, -0.25, -0.25]).value)
        ys = list(p('module_y', [0.25, -0.25, 0.25, -0.25]).value)
        if not len(names) == len(ids) == len(xs) == len(ys):
            raise ValueError('module_names / device_ids / module_x / module_y の数をそろえてください')
        self.kinematics = SwerveKinematics(
            [Module(n, x, y) for n, x, y in zip(names, xs, ys)],
            wheel_radius=p('wheel_radius', 0.05).value,
            max_wheel_speed=p('max_wheel_speed', 12.0).value)
        self.ids = ids
        self.cmd_timeout_s = p('cmd_timeout_s', 0.5).value
        self.state_timeout_s = p('state_timeout_s', 0.5).value
        self.use_cosine_scale = p('cosine_scale', False).value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_link').value
        self.publish_tf = p('publish_tf', True).value
        rate_hz = p('rate_hz', 50.0).value

        n = len(ids)
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None
        self.target_angles = [0.0] * n          # 最後に送った向き(止まるときはこの向きのまま)
        self.measured = [None] * n              # モジュールごとの (受け取った時刻, 向き, 速さ)
        self.pose = [0.0, 0.0, 0.0]             # オドメトリ: 地図上の x, y, 向き
        self.last_update = time.monotonic()

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.stopped = False        # 停止ボタン(/drive_stop)が押されている間は true
        self.create_subscription(Bool, robot_status.STOP_TOPIC, self.on_stop,
                                 robot_status.STOP_QOS)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self.publish_diagnostics)
        self.cmd_pubs = []
        for i, dev_id in enumerate(ids):
            self.cmd_pubs.append(self.create_publisher(
                Float32MultiArray, f'stm{dev_id}/command', 10))
            self.create_subscription(
                Float32MultiArray, f'stm{dev_id}/state',
                lambda msg, i=i: self.on_state(i, msg), 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'swerve/markers', 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_timer(1.0 / rate_hz, self.update)
        self.create_timer(5.0, self.check_states)
        self.get_logger().info(
            f'モジュール {dict(zip(names, ids))}、タイヤ半径 {self.kinematics.wheel_radius} m')

    # ---- 受け取る ----

    def on_cmd_vel(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.get_logger().error(f'cmd_vel に NaN か inf が入っているので無視します: {values}',
                                    throttle_duration_sec=1.0)
            return
        self.cmd = values
        self.cmd_time = time.monotonic()

    def on_state(self, index, msg):
        if len(msg.data) < 2 or not all(math.isfinite(v) for v in msg.data[:2]):
            # NaN が 1 回でも入ると、オドメトリの位置がずっと NaN になってしまう
            self.get_logger().warn(f'stm{self.ids[index]}/state を無視します: {list(msg.data)}',
                                   throttle_duration_sec=1.0)
            return
        self.measured[index] = (time.monotonic(), float(msg.data[0]), float(msg.data[1]))

    def fresh_states(self, now):
        """全モジュールの最新の状態が届いていれば (向き, 速さ) の並びを、そうでなければ None."""
        if any(m is None or now - m[0] > self.state_timeout_s for m in self.measured):
            return None
        return [m[1] for m in self.measured], [m[2] for m in self.measured]

    def on_stop(self, msg):
        if msg.data != self.stopped:
            self.get_logger().warn('停止ボタンが押されました。止まります' if msg.data
                                   else '停止ボタンが解除されました')
        self.stopped = msg.data

    def publish_diagnostics(self):
        """指令とモジュールの状態を /diagnostics に出す(1 秒ごと)."""
        now = time.monotonic()
        st = robot_status
        if self.stopped:
            cmd = st.make_status('swerve_drive: 指令', st.WARN, '停止ボタンで止めています')
        elif self.cmd_time is not None and now - self.cmd_time <= self.cmd_timeout_s:
            vx, vy, omega = self.cmd
            cmd = st.make_status('swerve_drive: 指令', st.OK, '/cmd_vel が届いています',
                                 前後=f'{vx:+.2f} m/s', 左右=f'{vy:+.2f} m/s',
                                 回転=f'{omega:+.2f} rad/s')
        else:
            cmd = st.make_status('swerve_drive: 指令', st.OK, '指令なし(止まっています)')
        late = self.late_devices(now)
        if late:
            parts = st.make_status('swerve_drive: モジュール', st.ERROR,
                                   f'{", ".join(late)} の状態が届いていません')
        else:
            parts = st.make_status('swerve_drive: モジュール', st.OK, '全部の状態が届いています')
        msg = DiagnosticArray(status=[cmd, parts])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(msg)

    # ---- 周期処理 ----

    def update(self):
        now = time.monotonic()
        dt = now - self.last_update
        self.last_update = now
        states = self.fresh_states(now)

        # 1. 指令: cmd_vel が途切れたら止まる(向きはそのまま)
        if self.stopped or self.cmd_time is None or now - self.cmd_time > self.cmd_timeout_s:
            vx = vy = omega = 0.0
        else:
            vx, vy, omega = self.cmd
        # 180° 反転の判断には、実際の向きがわかればそれを、なければ前回の目標を使う
        current = states[0] if states else self.target_angles
        commands = self.kinematics.inverse(vx, vy, omega, current)
        for i, c in enumerate(commands):
            speed = c.wheel_speed
            if self.use_cosine_scale and states:
                speed *= cosine_scale(c.angle, states[0][i])
            self.target_angles[i] = c.angle
            self.cmd_pubs[i].publish(Float32MultiArray(data=[c.angle, speed]))

        # 2. オドメトリ: 全モジュールの状態がそろっているときだけ進める
        if states:
            body = self.kinematics.forward(*states)
            self.pose = list(integrate_pose(*self.pose, *body, dt))
            self.publish_odometry(body)
        self.publish_markers(commands, states)

    def late_devices(self, now):
        return [f'stm{dev_id}' for dev_id, m in zip(self.ids, self.measured)
                if m is None or now - m[0] > self.state_timeout_s]

    def check_states(self):
        late = self.late_devices(time.monotonic())
        if late:
            self.get_logger().warn(
                f'{", ".join(late)} から状態が届いていません(オドメトリは止まっています)。'
                'stm_bridge か fake_modules が起動しているか確認してください')

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

    def publish_markers(self, commands, states):
        """各モジュールの位置に、目標の向き(赤)と実際の向き(緑)の矢印を出す."""
        markers = MarkerArray()
        for i, (module, c) in enumerate(zip(self.kinematics.modules, commands)):
            markers.markers.append(self.arrow(i * 2, module, c.angle, c.wheel_speed,
                                              (1.0, 0.2, 0.2, 0.6)))
            if states:
                markers.markers.append(self.arrow(i * 2 + 1, module, states[0][i],
                                                  states[1][i], (0.2, 1.0, 0.2, 1.0)))
        self.marker_pub.publish(markers)

    def arrow(self, marker_id, module, angle, wheel_speed, rgba):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.ns = 'modules'
        m.id = marker_id
        m.type = Marker.ARROW
        m.pose.position.x = module.x
        m.pose.position.y = module.y
        # 後ろ向きに回っているときは、実際に進む向きに矢印を向ける
        heading = angle if wheel_speed >= 0.0 else angle + math.pi
        m.pose.orientation.z = math.sin(heading / 2.0)
        m.pose.orientation.w = math.cos(heading / 2.0)
        m.scale.x = 0.05 + 0.2 * min(1.0, abs(wheel_speed) / self.kinematics.max_wheel_speed)
        m.scale.y = m.scale.z = 0.03
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.lifetime.nanosec = 500_000_000    # 届かなくなったら 0.5 秒で消す(古い矢印を残さない)
        return m


def main():
    rclpy.init()
    node = SwerveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
