"""
シミュレーションの世界のノード: ロボットの「本当の位置」を持ち、置き直しを受け付ける.

    ros2 run field_map sim_world_node --ros-args -p field:=example -p start:=red

受け取るもの:
    /odom          … 足回りのオドメトリ。これに滑りと向きのずれ(/sim/faults)を足して本当の位置を進める
    /sim/teleport  … (geometry_msgs/PoseStamped、地図の座標)ロボットをここに置き直す
    /sim/faults    … (std_msgs/String の JSON)slip、drift_deg_s を使う(swerve_drive/sim_faults.py)

出すもの:
    TF map → odom     … オドメトリの座標が、地図のどこにあるか(置き直したときに変わる)
    /sim/truth        … (geometry_msgs/PoseStamped)本当の位置。シミュレーション用の LiDAR とシミュレーターの画面が使う
    /sim/start_poses  … (std_msgs/String の JSON)フィールドのファイルの start_poses(赤・青のスタート位置など)

自分の位置を合わせる部品(AMCL)を使うときは、map → odom は AMCL が出すので、publish_map_to_odom:=false にする。
そのときは代わりに、置き直した位置を /initialpose(geometry_msgs/PoseWithCovarianceStamped)で AMCL に知らせる
(AMCL が起動したときも、起動の途中で取りこぼさないように、少し間をあけて何回か知らせる)。
"""

import json
import math
import os
import random

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from . import field as fm
from . import sim_truth as st

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimWorld(Node):

    def __init__(self):
        super().__init__('sim_world')
        p = self.declare_parameter
        self.path = fm.resolve_field_path(p('field', 'example').value)
        self.start = p('start', '').value
        self.publish_tf = p('publish_map_to_odom', True).value
        self.noise_std = p('odom_noise_std', 0.0).value     # 本当の動きに足すばらつき [m / 回]
        self.tracker = st.TruthTracker()
        self.faults = {'slip': 0.0, 'drift_deg_s': 0.0}
        self.poses, self.poses_stamp = {}, None
        self.last_odom_time = None
        self.sent_map_from_odom = None
        self.initial_subs = 0       # /initialpose を受け取るノード(AMCL)の数
        self.initial_due = []       # /initialpose を出す時刻 [s]

        self.tf = StaticTransformBroadcaster(self)
        self.truth_pub = self.create_publisher(PoseStamped, '/sim/truth', 10)
        self.poses_pub = self.create_publisher(String, '/sim/start_poses', LATCHED)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(PoseStamped, '/sim/teleport', self.on_teleport, 10)
        self.create_subscription(String, '/sim/faults', self.on_faults, LATCHED)
        self.send_map_to_odom()
        self.load_start_poses()
        if self.start:
            if self.start in self.poses:
                self.tracker.teleport(self.poses[self.start])
            else:
                self.get_logger().warn(f'スタート位置 {self.start!r} が {self.path} にありません')
        self.create_timer(0.05, self.publish_truth)
        self.create_timer(2.0, self.load_start_poses)
        if not self.publish_tf:
            self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
            self.create_timer(0.5, self.send_initial_pose)

    def load_start_poses(self):
        """フィールドのファイルが変わっていたら、スタート位置を読み直して出す."""
        try:
            stamp = os.stat(self.path).st_mtime_ns
        except OSError:
            return
        if stamp == self.poses_stamp:
            return
        self.poses_stamp = stamp
        try:
            import yaml
            with open(self.path, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            info = data.get('field') or {} if isinstance(data, dict) else {}
        except (OSError, ValueError, yaml.YAMLError):
            info = {}
        self.poses = st.start_poses(info)
        text = {k: [round(v[0], 4), round(v[1], 4), round(math.degrees(v[2]), 2)]
                for k, v in self.poses.items()}
        self.poses_pub.publish(String(data=json.dumps(text, ensure_ascii=False)))

    def on_faults(self, msg):
        try:
            data = json.loads(msg.data)
            self.faults = {'slip': float(data.get('slip', 0.0)),
                           'drift_deg_s': float(data.get('drift_deg_s', 0.0))}
        except (ValueError, TypeError, AttributeError):
            self.faults = {'slip': 0.0, 'drift_deg_s': 0.0}

    def on_odom(self, msg):
        pose = msg.pose.pose
        odom = (pose.position.x, pose.position.y, yaw_of(pose.orientation))
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.last_odom_time is None else max(0.0, min(0.5, now - self.last_odom_time))
        self.last_odom_time = now
        noise = None
        if self.noise_std > 0:
            s = self.noise_std
            noise = (lambda: (random.gauss(0, s), random.gauss(0, s), random.gauss(0, s * 0.5)))
        self.tracker.on_odom(odom, dt, self.faults['slip'],
                             math.radians(self.faults['drift_deg_s']), noise)
        self.send_map_to_odom()

    def on_teleport(self, msg):
        pose = msg.pose
        target = (pose.position.x, pose.position.y, yaw_of(pose.orientation))
        self.tracker.teleport(target)
        self.get_logger().info(f'ロボットを置き直しました: x {target[0]:+.2f} m、y {target[1]:+.2f} m、'
                               f'向き {math.degrees(target[2]):+.0f}°')
        self.send_map_to_odom()
        if not self.publish_tf:     # AMCL にも知らせる(すぐと、取りこぼし用に 1 秒後)
            self.initial_due += [self.seconds(), self.seconds() + 1.0]

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send_initial_pose(self):
        """AMCL に、本当の位置を最初の位置として知らせる(publish_map_to_odom:=false のとき)."""
        now = self.seconds()
        subs = self.initial_pub.get_subscription_count()
        if subs > self.initial_subs:            # AMCL が起動した
            self.initial_due += [now + 1.0, now + 3.0, now + 6.0]
        self.initial_subs = subs
        truth = self.tracker.truth
        if truth is None or not any(t <= now for t in self.initial_due):
            return
        self.initial_due = [t for t in self.initial_due if t > now]
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = truth[0], truth[1]
        msg.pose.pose.orientation.z = math.sin(truth[2] / 2)
        msg.pose.pose.orientation.w = math.cos(truth[2] / 2)
        # ばらつき: 位置 5 cm、向き 3°くらい(本当の位置なので小さく)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05 ** 2
        msg.pose.covariance[35] = math.radians(3.0) ** 2
        self.initial_pub.publish(msg)

    def send_map_to_odom(self):
        if not self.publish_tf:
            return
        x, y, yaw = self.tracker.map_from_odom
        if self.sent_map_from_odom == (x, y, yaw):
            return
        self.sent_map_from_odom = (x, y, yaw)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf.sendTransform(t)

    def publish_truth(self):
        truth = self.tracker.truth
        if truth is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y = truth[0], truth[1]
        msg.pose.orientation.z = math.sin(truth[2] / 2)
        msg.pose.orientation.w = math.cos(truth[2] / 2)
        self.truth_pub.publish(msg)


def main():
    rclpy.init()
    node = SimWorld()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
