"""
シミュレーション用の 2D LiDAR のノード(実機の LiDAR の代わり).

地図(/lidar_map: LiDAR の高さにかかる物だけの地図)とロボットの位置から、
LiDAR が見るはずの距離を計算して /scan(sensor_msgs/LaserScan)に出す。

    ros2 run field_map fake_lidar

ロボットの位置は、シミュレーションの本当の位置(/sim/truth。sim_world_node が出す)を使う。
届いていなければ TF(map → base_link)、それもなければ /odom を使う。
シミュレーターで置いた障害物(/sim/obstacles)と、LiDAR のばらつき(/sim/faults の lidar_noise)も反映する。
base_link → laser の TF(LiDAR の取り付け位置)も、このノードが出す。

パラメータ:
    rate_hz       … 1 秒に何回出すか(10)
    range_max     … 届く距離 [m](12)
    angle_step_deg… 光と光の間の角度 [度](1。360 本)
    noise_std     … 距離のばらつき [m](0.01)
    laser_x / laser_y / laser_yaw_deg … ロボットの中心から見た LiDAR の位置と向き
    map_topic     … 使う地図(lidar_map)
"""

import json
import math
import random
import time

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from . import lidar_sim as ls

MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FakeLidar(Node):

    def __init__(self):
        super().__init__('fake_lidar')
        p = self.declare_parameter
        self.range_max = p('range_max', 12.0).value
        self.step = math.radians(p('angle_step_deg', 1.0).value)
        self.count = max(1, int(round(2 * math.pi / self.step)))
        self.noise = p('noise_std', 0.01).value
        self.laser = (p('laser_x', 0.0).value, p('laser_y', 0.0).value,
                      math.radians(p('laser_yaw_deg', 0.0).value))
        self.frame = p('frame_id', 'laser').value
        self.base_frame = p('base_frame', 'base_link').value
        self.map_frame = p('map_frame', 'map').value
        self.grid = None
        self.odom_pose = None
        self.truth = None           # (受け取った時刻, (x, y, 向き))
        self.circles = []           # シミュレーターで置いた障害物 [(x, y, 半径)]
        self.noise_override = None  # /sim/faults の lidar_noise
        self.warned = False

        self.pub = self.create_publisher(LaserScan, 'scan', 10)
        self.create_subscription(OccupancyGrid, p('map_topic', 'lidar_map').value, self.on_map,
                                 MAP_QOS)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(PoseStamped, '/sim/truth', self.on_truth, 10)
        self.create_subscription(String, '/sim/obstacles', self.on_obstacles, MAP_QOS)
        self.create_subscription(String, '/sim/faults', self.on_faults, MAP_QOS)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publish_laser_tf()
        self.create_timer(1.0 / p('rate_hz', 10.0).value, self.tick)

    def publish_laser_tf(self):
        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = self.base_frame, self.frame
        t.transform.translation.x, t.transform.translation.y = self.laser[0], self.laser[1]
        t.transform.rotation.z = math.sin(self.laser[2] / 2)
        t.transform.rotation.w = math.cos(self.laser[2] / 2)
        self.static_tf.sendTransform(t)

    def on_map(self, msg):
        try:
            self.grid = ls.GridMap(msg.info.width, msg.info.height, msg.info.resolution,
                                   msg.info.origin.position.x, msg.info.origin.position.y,
                                   list(msg.data))
        except ValueError as e:
            self.get_logger().error(f'地図が使えません: {e}')

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.odom_pose = (pose.position.x, pose.position.y, yaw_of(pose.orientation))

    def on_truth(self, msg):
        p = msg.pose
        self.truth = (time.monotonic(), (p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_obstacles(self, msg):
        try:
            self.circles = [(float(c[0]), float(c[1]), max(0.01, float(c[2])))
                            for c in json.loads(msg.data)]
        except (ValueError, TypeError, IndexError):
            self.circles = []

    def on_faults(self, msg):
        try:
            noise = json.loads(msg.data).get('lidar_noise')
            self.noise_override = None if noise is None else max(0.0, float(noise))
        except (ValueError, TypeError, AttributeError):
            self.noise_override = None

    def robot_pose(self):
        """地図の座標でのロボットの位置と向き(本当の位置 → TF → /odom の順に使う)."""
        if self.truth is not None and time.monotonic() - self.truth[0] < 0.5:
            return self.truth[1]
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time(),
                                                timeout=Duration(seconds=0.0))
            tr = t.transform
            return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)
        except TransformException:
            return self.odom_pose

    def tick(self):
        pose = self.robot_pose()
        if self.grid is None or pose is None:
            if not self.warned:
                self.get_logger().info('地図(lidar_map)とロボットの位置を待っています')
                self.warned = True
            return
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy = self.laser[0], self.laser[1]
        lx, ly = x + c * ox - s * oy, y + s * ox + c * oy     # LiDAR の位置(地図の座標)
        ranges = ls.scan(self.grid, lx, ly, yaw + self.laser[2], -math.pi, self.step, self.count,
                         self.range_max, self.circles)
        noise = self.noise if self.noise_override is None else self.noise_override
        if noise > 0:
            ranges = [r + random.gauss(0.0, noise) if math.isfinite(r) else r for r in ranges]
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.angle_min, msg.angle_increment = -math.pi, self.step
        msg.angle_max = -math.pi + self.step * (self.count - 1)
        msg.range_min, msg.range_max = 0.05, float(self.range_max)
        msg.scan_time = 0.1
        msg.ranges = [float(max(0.0, r)) for r in ranges]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeLidar()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
