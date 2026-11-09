"""
独立ステアリング(スワーブ)の足回りの計算.

ROS に依存しないので、PC だけでテストできる(test/test_kinematics.py)。

座標の決まり(ROS の REP-103 と同じ):
    x … ロボットの前、y … ロボットの左、回転は上から見て反時計回りが正
    モジュールの向き(angle)… 0 が前、反時計回りが正 [rad]、-π〜π
    タイヤの速さ(wheel_speed)… タイヤの回転の速さ [rad/s]。正で angle の向きに進む

    逆運動学 … ロボット全体の速度 (vx, vy, ω) → 各モジュールの (向き, タイヤの速さ)
    順運動学 … 各モジュールの実際の (向き, タイヤの速さ) → ロボット全体の速度(オドメトリ用)

各モジュールの位置 (x, y) での速度は、ロボットの中心の速度に回転の分を足したもの:
    vx_i = vx − ω × y
    vy_i = vy + ω × x
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Module:
    """モジュール 1 個の取り付け位置."""

    name: str
    x: float    # ロボットの中心から前に何 m
    y: float    # ロボットの中心から左に何 m


@dataclass
class ModuleCommand:
    """モジュール 1 個への指令."""

    angle: float          # 目標の向き [rad]
    wheel_speed: float    # 目標のタイヤの速さ [rad/s]


def wrap_angle(a: float) -> float:
    """角度を -π〜π に収める."""
    return math.atan2(math.sin(a), math.cos(a))


class SwerveKinematics:
    """スワーブの逆運動学・順運動学."""

    def __init__(self, modules, wheel_radius: float, max_wheel_speed: float,
                 stop_threshold: float = 1e-3):
        """
        modules: Module の並び(例: 左前・右前・左後・右後).

        wheel_radius    … タイヤの半径 [m]
        max_wheel_speed … タイヤの速さの上限 [rad/s]。超えるときは全モジュールを同じ割合で遅くする
        stop_threshold  … これより遅い [m/s] モジュールは止まっているとみなし、向きを変えない
        """
        if not modules:
            raise ValueError('モジュールが 1 個もありません')
        if wheel_radius <= 0.0 or max_wheel_speed <= 0.0:
            raise ValueError('wheel_radius と max_wheel_speed は正の値にしてください')
        self.modules = list(modules)
        self.wheel_radius = wheel_radius
        self.max_wheel_speed = max_wheel_speed
        self.stop_threshold = stop_threshold

    def inverse(self, vx: float, vy: float, omega: float, current_angles):
        """
        ロボット全体の速度から、各モジュールの指令を求める.

        current_angles … 各モジュールの今の向き [rad]。次の 2 つに使う
          ・90° より大きく回るなら、向きを 180° 反転してタイヤを逆に回す(向きを変える量が減る)
          ・止まるときは向きをそのままにする(毎回 0° に戻ると、動き出しが遅くなる)
        """
        commands = []
        for module, current in zip(self.modules, current_angles):
            mvx = vx - omega * module.y
            mvy = vy + omega * module.x
            speed = math.hypot(mvx, mvy)    # [m/s]
            if speed < self.stop_threshold:
                commands.append(ModuleCommand(wrap_angle(current), 0.0))
                continue
            angle = math.atan2(mvy, mvx)
            wheel_speed = speed / self.wheel_radius
            if abs(wrap_angle(angle - current)) > math.pi / 2:
                angle = wrap_angle(angle + math.pi)
                wheel_speed = -wheel_speed
            commands.append(ModuleCommand(angle, wheel_speed))

        # どれかが上限を超えるなら、全部を同じ割合で遅くする(進む向きと回転の比を保つ)
        fastest = max(abs(c.wheel_speed) for c in commands)
        if fastest > self.max_wheel_speed:
            scale = self.max_wheel_speed / fastest
            for c in commands:
                c.wheel_speed *= scale
        return commands

    def forward(self, angles, wheel_speeds):
        """
        各モジュールの実際の向きとタイヤの速さから、ロボット全体の速度 (vx, vy, ω) を求める.

        モジュールが 3 個以上だと式が答えの数より多いので、最小二乗法で一番合う値を選ぶ
        (タイヤが少し滑っても、全体として平均した値になる)。
        """
        # 式: vx_i = vx − ω y_i、vy_i = vy + ω x_i を、未知数 (vx, vy, ω) について
        # 正規方程式 AᵀA [vx, vy, ω]ᵀ = Aᵀb にして解く
        n = len(self.modules)
        sx = sum(m.x for m in self.modules)
        sy = sum(m.y for m in self.modules)
        sr = sum(m.x * m.x + m.y * m.y for m in self.modules)
        bx = by = bw = 0.0
        for m, angle, w in zip(self.modules, angles, wheel_speeds):
            v = w * self.wheel_radius
            mvx = v * math.cos(angle)
            mvy = v * math.sin(angle)
            bx += mvx
            by += mvy
            bw += m.x * mvy - m.y * mvx
        ata = [[n, 0.0, -sy],
               [0.0, n, sx],
               [-sy, sx, sr]]
        return tuple(_solve3(ata, [bx, by, bw]))


def _solve3(a, b):
    """3 元の連立一次方程式をガウスの消去法で解く."""
    m = [row[:] + [rhs] for row, rhs in zip(a, b)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12:
            raise ValueError('モジュールの配置から回転が求められません(全部が同じ位置など)')
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(3):
            if r != col:
                f = m[r][col] / m[col][col]
                m[r] = [x - f * y for x, y in zip(m[r], m[col])]
    return [m[i][3] / m[i][i] for i in range(3)]


def cosine_scale(command_angle: float, measured_angle: float) -> float:
    """
    向きがまだ目標に合っていないときに、タイヤを遅くする割合(0〜1).

    向きのずれが 0° なら 1、90° 以上なら 0。向きを変えている途中に、
    違う方向へ全力で進んでしまうのを防ぐ。
    """
    return max(0.0, math.cos(wrap_angle(command_angle - measured_angle)))


def integrate_pose(x: float, y: float, yaw: float, vx: float, vy: float, omega: float,
                   dt: float):
    """ロボットの速度(ロボットから見た向き)で dt 秒進めた、地図上の位置と向きを返す."""
    mid = yaw + omega * dt / 2.0    # 途中の向きで回すと、回りながら進むときの誤差が小さい
    x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
    y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
    return x, y, wrap_angle(yaw + omega * dt)
