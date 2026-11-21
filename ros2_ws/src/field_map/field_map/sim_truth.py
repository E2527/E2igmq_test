"""
シミュレーションの「本当の位置」と、ロボットを置き直す計算.

ROS に依存しないので、PC だけでテストできる(test/test_sim_truth.py)。

    オドメトリ(/odom)… ロボットがタイヤの回転から「こう動いた」と思っている位置(odom の座標)
    本当の位置(truth)… シミュレーションの中で、ロボットが本当にいる位置(地図の座標)

実機ではタイヤが滑ったり、向きが少しずつずれたりするので、本当の位置とオドメトリはずれていく。
on_odom は、オドメトリの動きに滑り(slip)と向きのずれ(drift)を足して、本当の位置を進める。
シミュレーション用の LiDAR は本当の位置から見えるものを出すので、ずれると地図と点が合わなくなる
(自分の位置を合わせる AMCL を試すときに、そのずれを直せるかを確かめられる)。

置き直す(teleport)と、本当の位置もオドメトリの見かけの位置(地図の座標)も、そこに合わせる。
"""

import math

Pose = tuple    # (x, y, 向き [rad])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    """座標 a の中の位置 b を、a の外側の座標に直す(a ⊕ b)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    """逆の位置(元の位置と組み合わせると (0, 0, 0) になる)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


def relative(a, b):
    """位置 a から見た、位置 b."""
    return compose(inverse(a), b)


class TruthTracker:

    JUMP_M = 1.0        # 1 回にこれ以上動いたら、オドメトリがやり直されたとみなす

    def __init__(self):
        self.truth = None                   # 本当の位置(地図の座標)
        self.odom = None                    # 最後に届いたオドメトリ(odom の座標)
        self.map_from_odom = (0.0, 0.0, 0.0)    # TF の map → odom
        self.pending = None                 # オドメトリが届く前に頼まれた置き直し

    def on_odom(self, odom, dt=0.0, slip=0.0, drift_rad_s=0.0, noise=None):
        """
        オドメトリが届いた。本当の位置を進める.

        slip … 本当の動きが、オドメトリの動きよりこの割合だけ小さい(0〜0.9)
        drift_rad_s … 本当の向きが、1 秒にこれだけ余分に回る
        noise … 呼ぶと (dx, dy, dyaw) の小さなばらつきを返す関数(なければ足さない)
        """
        odom = tuple(float(v) for v in odom)
        if self.odom is None or self.truth is None:
            self.odom = odom
            self.truth = compose(self.map_from_odom, odom)
            if self.pending is not None:
                self.teleport(self.pending)
                self.pending = None
            return
        dx, dy, dyaw = relative(self.odom, odom)
        self.odom = odom
        if math.hypot(dx, dy) > self.JUMP_M:     # 足回りのノードが起動し直したなど
            self.truth = compose(self.map_from_odom, odom)
            return
        k = 1.0 - min(0.9, max(0.0, slip))
        step = (dx * k, dy * k, dyaw * k + drift_rad_s * dt)
        if noise is not None:
            nx, ny, nyaw = noise()
            step = (step[0] + nx, step[1] + ny, step[2] + nyaw)
        self.truth = compose(self.truth, step)

    def teleport(self, target):
        """ロボットを target(地図の座標)に置き直す。オドメトリの見かけの位置も合わせる."""
        target = (float(target[0]), float(target[1]), wrap(float(target[2])))
        if self.odom is None:
            self.pending = target
            return
        self.map_from_odom = compose(target, inverse(self.odom))
        self.truth = target

    def estimate(self):
        """オドメトリから見た位置(地図の座標)。届いていなければ None."""
        return None if self.odom is None else compose(self.map_from_odom, self.odom)

    def error(self):
        """本当の位置と、オドメトリから見た位置のずれ (距離 [m], 向き [rad])."""
        est = self.estimate()
        if est is None or self.truth is None:
            return 0.0, 0.0
        return math.hypot(est[0] - self.truth[0], est[1] - self.truth[1]), \
            wrap(est[2] - self.truth[2])


def start_poses(info):
    """スタート位置({名前: {x, y, yaw_deg}})を、{名前: (x, y, 向き [rad])} にする."""
    out = {}
    for name, value in (info.get('start_poses') or {}).items():
        try:
            out[str(name)] = (float(value.get('x', 0.0)), float(value.get('y', 0.0)),
                              math.radians(float(value.get('yaw_deg', 0.0))))
        except (AttributeError, TypeError, ValueError):
            continue
    return out
