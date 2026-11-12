"""
ロボットコンソールに表示するデータをまとめて持つ.

ROS にも画面にも依存しないので、PC だけでテストできる(test/test_console_state.py)。

    ROS のコールバック(別のスレッド)が on_cmd / on_odom / on_arrows / on_diagnostic で書き込み、
    画面(メインのスレッド)が snapshot() で、その時点の表示内容をまとめて受け取る。
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

    def clear_trail(self):
        with self._lock:
            self._trail.clear()

    # ---- 読む(画面から) ----

    def snapshot(self, now) -> Snapshot:
        with self._lock:
            snap = Snapshot(pose=self._pose, trail=list(self._trail), drive=self._drive)
            if self._cmd_time is not None and now - self._cmd_time < self.cmd_timeout_s:
                snap.command = self._cmd
            if self._odom_time is not None and now - self._odom_time < self.odom_timeout_s:
                snap.actual = self._actual
            if self._arrows_time is not None and now - self._arrows_time < self.odom_timeout_s:
                snap.targets = list(self._targets)
                snap.actuals = list(self._actuals)
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
