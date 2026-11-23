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
FIX_MIN_M = 0.002       # map → odom がこれ以上変わったら、AMCL が位置を直したとみなす [m]
FIX_MIN_RAD = math.radians(0.2)
NAV_LOG_MAX = 6         # 自動操縦の警告を、新しいものから何個まで残すか
NAV_LOG_S = 60.0        # 何秒前までの警告を出すか
MUX_TIMEOUT_S = 1.0     # 指令のまとめ役(cmd_vel_mux)の知らせがこれ以上来なければ、分からないとみなす

# 自動操縦の進み具合(nav_phase)
NAV_IDLE, NAV_SENDING, NAV_RUNNING, NAV_CANCELING = 'idle', 'sending', 'running', 'canceling'
NAV_SUCCEEDED, NAV_CANCELED, NAV_FAILED = 'succeeded', 'canceled', 'failed'
NAV_TEXT = {NAV_IDLE: '待機中', NAV_SENDING: 'ゴールを送っています', NAV_RUNNING: '走っています',
            NAV_CANCELING: 'やめています', NAV_SUCCEEDED: '着きました', NAV_CANCELED: 'やめました',
            NAV_FAILED: '行けませんでした'}

# 自動操縦で、ロボットの向きをいつ変えるか
TURN_WHILE_MOVING = 'moving'    # 走りながら回る
TURN_ON_ARRIVAL = 'arrival'     # 向きを変えずに走り、着いてからその場で回る
TURN_TEXT = {TURN_WHILE_MOVING: '走りながら回る', TURN_ON_ARRIVAL: '着いてから回る'}
# Nav2 の道に沿って走る部品(robot_nav の config/nav2_params.yaml の controller_plugins)
CONTROLLER_KEEP = 'FollowPath'              # ゴールの近くまで向きを変えない
CONTROLLER_TURNING = 'FollowPathTurning'    # ゴールの 2 m 手前から、走りながらゴールの向きに回る
TURN_DONE_RAD = math.radians(3.0)           # 向きがこれ以下しか違わなければ、着いてから回らない


def goal_stages(mode, points, current_yaw):
    """
    経由地点の並び(最後がゴール)を、Nav2 に順番に送るものに分ける.

    points は [(x, y, 向き)]。返すのは [(道に沿って走る部品の名前, [(x, y, 向き)])]。
    点が 1 つなら navigate_to_pose、2 つ以上なら navigate_through_poses で送る。
    向きを使うのはゴール(最後の点)だけ(経由地点は通り過ぎるだけなので、向きは気にしない)。
        着いてから回る: 1. 今の向きのまま全部の点を通ってゴールまで走る → 2. その場でゴールの向きに回る
        走りながら回る: 1 回だけ(ゴールの手前から、走りながらゴールの向きに回る部品で)
    """
    points = [tuple(float(v) for v in p) for p in points]
    gx, gy, gyaw = points[-1]
    if mode == TURN_ON_ARRIVAL:
        stages = [(CONTROLLER_KEEP, [(x, y, current_yaw) for x, y, _ in points])]
        diff = math.atan2(math.sin(gyaw - current_yaw), math.cos(gyaw - current_yaw))
        if abs(diff) > TURN_DONE_RAD:
            stages.append((CONTROLLER_KEEP, [(gx, gy, gyaw)]))
        return stages
    return [(CONTROLLER_TURNING, points)]


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

    def clearance(self, x, y, limit):
        """点 (x, y) から、いちばん近いふさがっているマス(と地図の外)までの距離 [m]。limit まで調べる."""
        res = self.resolution
        col, row = math.floor((x - self.origin_x) / res), math.floor((y - self.origin_y) / res)
        reach = int(math.ceil(limit / res)) + 1
        best = limit
        for r in range(row - reach, row + reach + 1):
            for c in range(col - reach, col + reach + 1):
                inside = 0 <= c < self.width and 0 <= r < self.height
                if inside and not 50 <= self.cells[r * self.width + c] <= 100:
                    continue
                # マスの四角の中で、いちばん近い点までの距離
                cx = min(max(x, self.origin_x + c * res), self.origin_x + (c + 1) * res)
                cy = min(max(y, self.origin_y + r * res), self.origin_y + (r + 1) * res)
                best = min(best, math.hypot(cx - x, cy - y))
        return best


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
    nav_ready: bool = False             # 自動操縦(Nav2)が起動している
    nav_phase: str = NAV_IDLE           # 自動操縦の進み具合(NAV_*)
    nav_text: str = ''                  # 補足(行けなかった理由など)
    nav_remaining: float = None         # ゴールまでの残りの距離 [m](分からなければ None)
    goal: tuple = None                  # 自動操縦のゴール(地図の座標。なければ None)
    route: list = field(default_factory=list)       # 自動操縦の経由地点 [(x, y, 向き)](最後がゴール)
    nav_points_left: int = None         # まだ通っていない点の数(分からなければ None)
    nav_recoveries: int = 0             # 行き詰まって、回る・待つ・下がるをした回数(今のゴールで)
    nav_logs: list = field(default_factory=list)    # Nav2 の警告 [(何秒前, 部品の名前, 文)](新しい順)
    plan: list = field(default_factory=list)        # 自動操縦が探した道 [(x, y)](地図の座標)
    amcl_age: float = None              # AMCL が最後に位置を合わせてから何秒(動いていなければ None)
    amcl_spread: tuple = None           # そのときの位置のばらつき (m, rad)(標準偏差)
    last_fix: tuple = None              # 最後に直した量 (距離 m, 向き rad, 何秒前)。なければ None
    params: dict = field(default_factory=dict)      # STM32 のゲイン {ID: {番号: (値, 結果, 何秒前)}}
    mux_source: str = None              # 今 /cmd_vel に通している指令('teleop' / 'nav' / ''。分からなければ None)

    @property
    def navigating(self):
        """自動操縦の途中(ゴールを送ってから、終わるまで)."""
        return self.nav_phase in (NAV_SENDING, NAV_RUNNING, NAV_CANCELING)

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
        self._nav_ready = False
        self._nav = (NAV_IDLE, '', None)        # (進み具合, 補足, 残りの距離)
        self._goal = None
        self._route = []
        self._points_left = None
        self._recoveries = 0
        self._nav_logs = deque(maxlen=NAV_LOG_MAX)
        self._plan = []
        self._mux = None
        self._mux_time = None
        self._amcl = None               # (時刻, (m, rad))
        self._fix = None                # (時刻, 距離, 向き)
        self._odom_in_map_known = False     # TF の map → odom が 1 回でも届いた
        self._params = {}               # (ID, 番号) → (値, 結果, 時刻)

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

    def set_odom_in_map(self, x, y, yaw, now=None):
        """
        TF の map → odom(自分の位置を合わせる部品が出す).

        now を渡すと(TF が届いたとき)、前に届いた値から変わっていれば「直した」と記録する。
        """
        with self._lock:
            ox, oy, oyaw = self._odom_in_map
            dist = math.hypot(x - ox, y - oy)
            dyaw = math.atan2(math.sin(yaw - oyaw), math.cos(yaw - oyaw))
            if now is not None and self._odom_in_map_known and \
                    (dist >= FIX_MIN_M or abs(dyaw) >= FIX_MIN_RAD):
                self._fix = (now, dist, dyaw)
            self._odom_in_map_known = self._odom_in_map_known or now is not None
            self._odom_in_map = (x, y, yaw)

    def on_param_reply(self, dev_id, param_id, status, value, now):
        """STM32 からゲインの読み書きの返事が届いた(stm<ID>/param_reply)."""
        with self._lock:
            self._params[(int(dev_id), int(param_id))] = (float(value), int(status), now)

    def on_amcl(self, spread_m, spread_rad, now):
        """AMCL が位置を合わせた(/amcl_pose)。spread は位置と向きのばらつき(標準偏差)."""
        with self._lock:
            self._amcl = (now, (float(spread_m), float(spread_rad)))

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

    def set_nav_ready(self, ready):
        with self._lock:
            self._nav_ready = bool(ready)

    def set_nav(self, phase, text='', remaining=None, goal=False):
        """
        自動操縦の進み具合.

        goal を渡すと、ゴールも変える。(x, y, 向き) 1 つか、経由地点の並び(最後がゴール)。None で消す。
        """
        with self._lock:
            self._nav = (phase, text, remaining)
            if remaining is None:
                self._points_left = None
            if phase == NAV_SENDING:
                self._recoveries = 0
            if goal is not False:
                route = [] if goal is None else (
                    [tuple(goal)] if not isinstance(goal[0], (tuple, list)) else
                    [tuple(p) for p in goal])
                self._route = route
                self._goal = route[-1] if route else None
            if phase in (NAV_SUCCEEDED, NAV_CANCELED, NAV_FAILED):
                self._plan = []                 # 終わったら、道の表示を消す

    def on_nav_feedback(self, remaining, points_left=None, recoveries=None):
        """走っている途中の知らせ(ゴールまでの残りの距離 [m]、まだ通っていない点の数、行き詰まった回数)."""
        with self._lock:
            if self._nav[0] == NAV_RUNNING:
                self._nav = (NAV_RUNNING, self._nav[1], float(remaining))
                self._points_left = None if points_left is None else int(points_left)
                if recoveries is not None:
                    self._recoveries = int(recoveries)

    def on_nav_log(self, name, text, now):
        """Nav2 の部品が出した警告・エラー(/rosout)."""
        with self._lock:
            if self._nav_logs and self._nav_logs[-1][1:] == (name, text):
                self._nav_logs[-1] = (now, name, text)      # 同じ文が続くときは、1 つにまとめる
            else:
                self._nav_logs.append((now, name, text))

    def on_plan(self, points):
        """自動操縦が探した道(/plan。地図の座標の [(x, y)])."""
        with self._lock:
            if self._nav[0] in (NAV_SENDING, NAV_RUNNING):
                self._plan = [(float(x), float(y)) for x, y in points]

    def on_mux(self, source, now):
        """cmd_vel_mux が今通している指令('teleop' / 'nav' / '')."""
        with self._lock:
            self._mux = source
            self._mux_time = now

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
            snap.nav_ready = self._nav_ready
            snap.nav_phase, snap.nav_text, snap.nav_remaining = self._nav
            snap.goal = self._goal
            snap.route = list(self._route)
            snap.nav_points_left = self._points_left
            snap.nav_recoveries = self._recoveries
            snap.nav_logs = [(now - t, name, text) for t, name, text in reversed(self._nav_logs)
                             if now - t < NAV_LOG_S]
            snap.plan = list(self._plan)
            for (dev_id, param_id), (value, status, t) in self._params.items():
                snap.params.setdefault(dev_id, {})[param_id] = (value, status, now - t)
            if self._amcl is not None:
                snap.amcl_age = now - self._amcl[0]
                snap.amcl_spread = self._amcl[1]
            if self._fix is not None:
                snap.last_fix = (self._fix[1], self._fix[2], now - self._fix[0])
            if self._mux_time is not None and now - self._mux_time < MUX_TIMEOUT_S:
                snap.mux_source = self._mux
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
