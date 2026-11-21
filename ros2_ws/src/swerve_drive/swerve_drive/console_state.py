"""
ロボットコンソールに表示するデータをまとめて持つ.

ROS にも画面にも依存しないので、PC だけでテストできる(test/test_console_state.py)。

    ROS のコールバック(別のスレッド)が on_cmd / on_odom / on_arrows / on_diagnostic /
    on_map / on_scan / set_odom_in_map で書き込み、
    画面(メインのスレッド)が snapshot() で、その時点の表示内容をまとめて受け取る。

座標: /odom はオドメトリの座標(odom)、地図と表示は地図の座標(map)。
      odom が map のどこにあるか(TF の map → odom)を odom_in_map に持ち、表示のときに直す。
      自分の位置を合わせる部品(AMCL など)がなければ、map と odom は同じとみなす。
"""

from collections import deque
from dataclasses import dataclass, field
import math
import threading

# 状態の重さ(diagnostic_msgs/DiagnosticStatus と同じ数。STALE は「届いていない」)
OK, WARN, ERROR, STALE = 0, 1, 2, 3
LEVEL_TEXT = {OK: 'OK', WARN: '注意', ERROR: 'エラー', STALE: '届いていない'}

TRAIL_STEP = 0.02       # これ以上動いたら、軌跡に点を足す [m]
TRAIL_MAX = 4000        # 軌跡の点の数の上限(古い点から消える)
SCAN_TIMEOUT_S = 1.0    # LiDAR の点がこれ以上来なければ、表示しない
HISTORY_S = 20.0        # グラフに残す長さ [s]
RATE_WINDOW_S = 1.0     # 通信の回数を数える長さ [s]


@dataclass(frozen=True)
class StmRow:
    """通信の一覧の 1 行(STM32 1 枚分)."""

    dev_id: int
    command: tuple      # 最後の指令(届いていなければ ())
    command_rate: float     # 指令の回数 [回/秒]
    command_age: float      # 最後の指令から何秒たったか(届いていなければ None)
    state: tuple
    state_rate: float
    state_age: float


@dataclass(frozen=True)
class Sample:
    """グラフ用に、ある時刻の値をまとめたもの."""

    t: float
    command: tuple          # 指令の速さ (前後, 左右, 回転)
    actual: tuple           # 実際の速さ(届いていなければ None)
    stm: dict               # {ID: (指令の値, 状態の値)}


@dataclass(frozen=True)
class MapData:
    """地図(nav_msgs/OccupancyGrid と同じ中身)."""

    width: int
    height: int
    resolution: float   # 1 マスの大きさ [m]
    origin_x: float     # 左下の角の位置 [m]
    origin_y: float
    cells: bytes        # マスの値(0 通れる、100 ふさがっている、255 = -1 分からない)
    version: int = 0    # 届くたびに増える(画面が絵を作り直すかの判定用)

    @property
    def bounds(self):
        """(x_min, x_max, y_min, y_max)."""
        return (self.origin_x, self.origin_x + self.width * self.resolution,
                self.origin_y, self.origin_y + self.height * self.resolution)


def compose(frame, x, y):
    """座標 frame = (x, y, 向き) の中の点 (x, y) を、その外側の座標に直す."""
    fx, fy, fyaw = frame
    c, s = math.cos(fyaw), math.sin(fyaw)
    return fx + c * x - s * y, fy + s * x + c * y


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, laser_pose):
    """距離の並び(LaserScan)を、ロボットから見た点 [(x, y)] にする(laser_pose は LiDAR の取り付け)."""
    points = []
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and range_min <= r <= range_max):
            continue
        a = angle_min + i * angle_increment
        points.append(compose(laser_pose, r * math.cos(a), r * math.sin(a)))
    return points


def compose_pose(frame, pose):
    """座標 frame の中の位置と向き pose を、その外側の座標に直す."""
    x, y = compose(frame, pose[0], pose[1])
    return x, y, math.atan2(math.sin(frame[2] + pose[2]), math.cos(frame[2] + pose[2]))


@dataclass(frozen=True)
class Arrow:
    """足回りの矢印 1 本(ロボットから見た位置と向き)."""

    x: float            # 前に何 m
    y: float            # 左に何 m
    heading: float      # 向き [rad](0 が前、反時計回りが正)
    length: float       # 長さ(速いほど長い。足回りのノードが決めた値)


@dataclass(frozen=True)
class Row:
    """状態の一覧の 1 行."""

    name: str
    level: int          # OK / WARN / ERROR / STALE
    message: str
    values: str         # 「受信 100 回/秒」のような補足


@dataclass
class Snapshot:
    """ある時点の表示内容."""

    command: tuple = (0.0, 0.0, 0.0)    # 指令の速さ (前後, 左右, 回転)。途切れていたら 0
    actual: tuple = None                # 実際の速さ。/odom が届いていなければ None
    pose: tuple = (0.0, 0.0, 0.0)       # 地図上の位置と向き (x, y, 向き)
    trail: list = field(default_factory=list)       # 走った軌跡 [(x, y), ...]
    targets: list = field(default_factory=list)     # 目標の矢印
    actuals: list = field(default_factory=list)     # 実際の矢印
    drive: str = ''                     # 'swerve' / 'omni' / ''(まだ分からない)
    rows: list = field(default_factory=list)        # 状態の一覧
    level: int = STALE                  # 全体の状態
    summary: str = ''                   # 全体の状態の文
    stopped: bool = False               # 停止ボタンで止まっている
    capped: bool = False                # 実際の速さが指令に届いていない(頭打ち)
    grid: MapData = None                # 地図(届いていなければ None)
    scan: list = field(default_factory=list)        # LiDAR の点 [(x, y)](ロボットから見た位置)
    odom_in_map: tuple = (0.0, 0.0, 0.0)    # odom の座標が、地図のどこにあるか
    stm_rows: list = field(default_factory=list)    # 通信の一覧
    history: list = field(default_factory=list)     # グラフ用の値 [Sample]
    truth: tuple = None                 # シミュレーションの本当の位置(地図の座標。なければ None)

    @property
    def map_pose(self):
        """地図の座標での、ロボットの位置と向き."""
        return compose_pose(self.odom_in_map, self.pose)

    def map_trail(self):
        """地図の座標での軌跡."""
        return [compose(self.odom_in_map, x, y) for x, y in self.trail]


class ConsoleState:

    def __init__(self, cmd_timeout_s=0.5, odom_timeout_s=1.0, stale_s=3.0):
        self.cmd_timeout_s = cmd_timeout_s      # 指令がこれ以上来なければ、止まっているとみなす
        self.odom_timeout_s = odom_timeout_s    # /odom や矢印がこれ以上来なければ、表示しない
        self.stale_s = stale_s                  # 状態がこれ以上来なければ「届いていない」
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_time = None
        self._actual = (0.0, 0.0, 0.0)
        self._pose = (0.0, 0.0, 0.0)
        self._odom_time = None
        self._trail = deque(maxlen=TRAIL_MAX)
        self._targets = []
        self._actuals = []
        self._arrows_time = None
        self._drive = ''
        self._items = {}                # 名前 → (時刻, 重さ, 文, 補足)
        self._stop_sent = False         # このコンソールが出した停止の指示
        self._stop_sent_time = None
        self._stop_reported = False     # 足回りが「止まっている」と知らせてきた
        self._stop_reported_time = None
        self._map = None
        self._scan = []
        self._scan_time = None
        self._odom_in_map = (0.0, 0.0, 0.0)
        self._stm = {}                  # (ID, 'command' / 'state') → (時刻, 値, 届いた時刻の並び)
        self._history = deque()
        self._truth = None
        self._truth_time = None

    # ---- 書き込む(ROS のコールバックから) ----

    def on_cmd(self, vx, vy, omega, now):
        with self._lock:
            self._cmd = (vx, vy, omega)
            self._cmd_time = now

    def on_odom(self, x, y, yaw, vx, vy, omega, now):
        with self._lock:
            self._pose = (x, y, yaw)
            self._actual = (vx, vy, omega)
            self._odom_time = now
            if not self._trail or math.hypot(x - self._trail[-1][0],
                                             y - self._trail[-1][1]) >= TRAIL_STEP:
                self._trail.append((x, y))

    def on_arrows(self, drive, targets, actuals, now):
        """足回りの矢印(swerve/markers か omni/markers)。drive は 'swerve' か 'omni'."""
        with self._lock:
            self._drive = drive
            self._targets = list(targets)
            self._actuals = list(actuals)
            self._arrows_time = now

    def on_diagnostic(self, name, level, message, values, now):
        """/diagnostics の 1 項目。values は {名前: 値}."""
        with self._lock:
            values = dict(values)
            # 足回りは「止まっているか」を stopped=true / false で知らせる(表示はしない)
            reported = values.pop('stopped', None)
            if reported is not None:
                self._stop_reported = reported == 'true'
                self._stop_reported_time = now
            text = '   '.join(f'{k} {v}' for k, v in values.items())
            self._items[name] = (now, int(level), message, text)

    def set_stop(self, stop, now):
        """停止ボタンを押した(このコンソールから、またはほかのコンソールから)."""
        with self._lock:
            self._stop_sent = stop
            self._stop_sent_time = now

    def on_map(self, width, height, resolution, origin_x, origin_y, cells):
        """地図(/map)。cells は -1〜100 の並び(bytes なら -1 は 255)."""
        with self._lock:
            version = self._map.version + 1 if self._map else 1
            data = bytes(v & 0xFF for v in cells) if not isinstance(cells, bytes) else cells
            self._map = MapData(int(width), int(height), float(resolution), float(origin_x),
                                float(origin_y), data, version)

    def on_scan(self, points, now):
        """ロボット(base_link)から見た、LiDAR の点 [(x, y)]."""
        with self._lock:
            self._scan = list(points)
            self._scan_time = now

    def set_odom_in_map(self, x, y, yaw):
        """TF の map → odom(自分の位置を合わせる部品が出す)."""
        with self._lock:
            self._odom_in_map = (x, y, yaw)

    def on_stm(self, dev_id, kind, values, now):
        """stm<ID>/command か stm<ID>/state が届いた(kind は 'command' か 'state')."""
        with self._lock:
            key = (int(dev_id), kind)
            times = self._stm[key][2] if key in self._stm else deque()
            times.append(now)
            while times and now - times[0] > RATE_WINDOW_S:
                times.popleft()
            self._stm[key] = (now, tuple(float(v) for v in values), times)

    def set_truth(self, x, y, yaw, now):
        """シミュレーションの本当の位置(/sim/truth)."""
        with self._lock:
            self._truth = (x, y, yaw)
            self._truth_time = now

    def sample(self, now):
        """グラフ用に、今の値を 1 つ記録する(画面が定期的に呼ぶ)."""
        snap = self.snapshot(now, with_history=False)
        stm = {r.dev_id: (r.command, r.state) for r in snap.stm_rows}
        with self._lock:
            self._history.append(Sample(now, snap.command, snap.actual, stm))
            while self._history and now - self._history[0].t > HISTORY_S:
                self._history.popleft()

    def clear_trail(self):
        with self._lock:
            self._trail.clear()

    # ---- 読む(画面から) ----

    def snapshot(self, now, with_history=True) -> Snapshot:
        with self._lock:
            snap = Snapshot(pose=self._pose, trail=list(self._trail), drive=self._drive)
            if self._cmd_time is not None and now - self._cmd_time < self.cmd_timeout_s:
                snap.command = self._cmd
            if self._odom_time is not None and now - self._odom_time < self.odom_timeout_s:
                snap.actual = self._actual
            if self._arrows_time is not None and now - self._arrows_time < self.odom_timeout_s:
                snap.targets = list(self._targets)
                snap.actuals = list(self._actuals)
            snap.grid = self._map
            snap.odom_in_map = self._odom_in_map
            if self._scan_time is not None and now - self._scan_time < SCAN_TIMEOUT_S:
                snap.scan = list(self._scan)
            if self._truth_time is not None and now - self._truth_time < SCAN_TIMEOUT_S:
                snap.truth = self._truth
            snap.stm_rows = self._stm_rows(now)
            if with_history:
                snap.history = list(self._history)
            snap.stopped = self._stopped(now)
            snap.capped = self._capped(snap.command, snap.actual)
            for name in sorted(self._items):
                received, level, message, text = self._items[name]
                if now - received > self.stale_s:
                    level = STALE
                    message = f'{now - received:.0f} 秒前から届いていません'
                snap.rows.append(Row(name, level, message, text))
            snap.level, snap.summary = self._summary(snap)
            return snap

    def _stm_rows(self, now):
        rows = []
        for dev_id in sorted({k[0] for k in self._stm}):
            parts = []
            for kind in ('command', 'state'):
                entry = self._stm.get((dev_id, kind))
                if entry is None:
                    parts += [(), 0.0, None]
                    continue
                t, values, times = entry
                recent = sum(1 for x in times if now - x <= RATE_WINDOW_S)
                parts += [values, recent / RATE_WINDOW_S, now - t]
            rows.append(StmRow(dev_id, *parts))
        return rows

    def _stopped(self, now):
        # ボタンを押した直後は、足回りの知らせ(1 秒ごと)がまだ古いので、自分の指示を信じる
        recent = self._stop_sent_time is not None and now - self._stop_sent_time < 1.5
        fresh = (self._stop_reported_time is not None
                 and now - self._stop_reported_time < self.stale_s)
        if fresh and not recent:
            return self._stop_reported
        return self._stop_sent

    @staticmethod
    def _capped(command, actual):
        if actual is None:
            return False
        # どれか 1 つでも、実際が指令の 8 割に届いていなければ「頭打ち」
        return any(abs(c) > 0.05 and abs(a) < 0.8 * abs(c) for c, a in zip(command, actual))

    @staticmethod
    def _summary(snap):
        if snap.stopped:
            return WARN, '停止中'
        bad = [r for r in snap.rows if r.level in (ERROR, STALE)]
        if bad:
            return ERROR, f'エラー {len(bad)} 件'
        if any(r.level == WARN for r in snap.rows):
            return WARN, '注意があります'
        if snap.capped:
            return WARN, '頭打ち(指令より遅い)'
        if not snap.rows:
            return STALE, 'ロボットからの知らせを待っています'
        return OK, 'すべて OK'
