"""
オムニホイールの足回りの計算.

ROS に依存しないので、PC だけでテストできる(test/test_kinematics.py)。
3 輪でも 4 輪でも、ホイールの位置と向きを決めれば同じ式で計算できる。

座標の決まり(ROS の REP-103 と同じ):
    x … ロボットの前、y … ロボットの左、回転は上から見て反時計回りが正

ホイール 1 個の決まり:
    (x, y)    … ロボットの中心からの位置 [m]
    direction … タイヤが正に回ったときに、床を押して進む向き [rad](0 が前、反時計回りが正)
                オムニホイールは横向きには自由に滑るので、この向きの速さだけを出せる

ホイールの位置での速度は、ロボットの中心の速度に回転の分を足したもの:
    vx_i = vx − ω × y
    vy_i = vy + ω × x
そのうち、ホイールの向きの成分がタイヤの速さになる:
    v_i = vx_i cos(direction) + vy_i sin(direction)
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Wheel:
    """ホイール 1 個の取り付け位置と向き."""

    name: str
    x: float            # ロボットの中心から前に何 m
    y: float            # ロボットの中心から左に何 m
    direction: float    # タイヤが正に回ったときに進む向き [rad]


class OmniKinematics:
    """オムニホイールの逆運動学・順運動学."""

    def __init__(self, wheels, wheel_radius: float, max_wheel_speed: float):
        """
        wheels: Wheel の並び(3 個以上).

        wheel_radius    … タイヤの半径 [m]
        max_wheel_speed … タイヤの速さの上限 [rad/s]。超えるときは全部を同じ割合で遅くする
        """
        if len(wheels) < 3:
            raise ValueError('オムニホイールは 3 個以上必要です(2 個では全方向に動けない)')
        if wheel_radius <= 0.0 or max_wheel_speed <= 0.0:
            raise ValueError('wheel_radius と max_wheel_speed は正の値にしてください')
        self.wheels = list(wheels)
        self.wheel_radius = wheel_radius
        self.max_wheel_speed = max_wheel_speed
        # ホイールごとの式 v_i = a_x vx + a_y vy + a_w ω の係数
        self.rows = [(math.cos(w.direction), math.sin(w.direction),
                      w.x * math.sin(w.direction) - w.y * math.cos(w.direction))
                     for w in self.wheels]
        # 順運動学で使う正規方程式の左辺(配置で決まるので最初に 1 回だけ求める)
        self.ata = [[sum(r[i] * r[j] for r in self.rows) for j in range(3)] for i in range(3)]
        _solve3(self.ata, [0.0, 0.0, 0.0])   # 配置がおかしければ、ここで分かる

    def inverse(self, vx: float, vy: float, omega: float):
        """ロボット全体の速度から、各タイヤの速さ [rad/s] を求める."""
        speeds = [(ax * vx + ay * vy + aw * omega) / self.wheel_radius
                  for ax, ay, aw in self.rows]
        # どれかが上限を超えるなら、全部を同じ割合で遅くする(進む向きと回転の比を保つ)
        fastest = max(abs(s) for s in speeds)
        if fastest > self.max_wheel_speed:
            scale = self.max_wheel_speed / fastest
            speeds = [s * scale for s in speeds]
        return speeds

    def forward(self, wheel_speeds):
        """
        各タイヤの実際の速さ [rad/s] から、ロボット全体の速度 (vx, vy, ω) を求める.

        4 輪だと式が答えの数より多いので、最小二乗法で一番合う値を選ぶ
        (タイヤが少し滑っても、全体として平均した値になる)。
        """
        b = [0.0, 0.0, 0.0]
        for row, w in zip(self.rows, wheel_speeds):
            v = w * self.wheel_radius
            for i in range(3):
                b[i] += row[i] * v
        return tuple(_solve3(self.ata, b))


def _solve3(a, b):
    """3 元の連立一次方程式をガウスの消去法で解く."""
    m = [row[:] + [rhs] for row, rhs in zip(a, b)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-9:
            raise ValueError('ホイールの配置では全方向に動けません(向きや位置を確認してください)')
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(3):
            if r != col:
                f = m[r][col] / m[col][col]
                m[r] = [x - f * y for x, y in zip(m[r], m[col])]
    return [m[i][3] / m[i][i] for i in range(3)]


def wrap_angle(a: float) -> float:
    """角度を -π〜π に収める."""
    return math.atan2(math.sin(a), math.cos(a))


def integrate_pose(x: float, y: float, yaw: float, vx: float, vy: float, omega: float,
                   dt: float):
    """ロボットの速度(ロボットから見た向き)で dt 秒進めた、地図上の位置と向きを返す."""
    mid = yaw + omega * dt / 2.0    # 途中の向きで回すと、回りながら進むときの誤差が小さい
    x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
    y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
    return x, y, wrap_angle(yaw + omega * dt)
