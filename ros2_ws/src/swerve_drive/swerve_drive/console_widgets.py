"""
ロボットコンソールの画面の部品(PyQt5).

    Card        … 角の丸い枠と見出し。下の部品を 1 つ入れる
    RobotView   … ロボットを上から見た図(各モジュール / ホイールの矢印、進む向き、回転)
    TrailView   … 走った軌跡(地図)
    SpeedGauges … 前後・左右・回転の、指令の速さと実際の速さ
    KeyPad      … WASD のキー(押しているキーが光る。マウスで押してもよい)
    StatusList  … 状態の一覧(/diagnostics)

どの部品も set_snapshot() で console_state.Snapshot を受け取って、自分で絵を描く。

ウィンドウの大きさが変わったときは、ウィンドウが各部品の set_scale(倍率) を呼ぶ。
絵は「倍率 1 のときの大きさ」で描いておき、まるごと拡大・縮小する
(文字・線・すき間の比が、どの大きさでも同じになる)。
"""

import math

from PyQt5.QtCore import pyqtSignal, QPointF, QRectF, Qt
from PyQt5.QtGui import QFontMetricsF, QLinearGradient, QPainter, QPainterPath, QPolygonF
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from . import console_state as cs
from . import console_theme as th


class Card(QFrame):
    """角の丸い枠と見出し."""

    def __init__(self, title, tag, body, action=None):
        super().__init__()
        self.setObjectName('card')
        self.body = body
        self.box = QVBoxLayout(self)
        self.head = QHBoxLayout()
        self.title_label = QLabel(title)
        self.title_label.setStyleSheet(f'color: {th.TEXT};')
        self.tag_label = QLabel(tag)
        self.tag_label.setStyleSheet(f'color: {th.FAINT};')
        self.head.addWidget(self.title_label)
        self.head.addWidget(self.tag_label)
        self.head.addStretch(1)
        if action is not None:
            self.head.addWidget(action)
        self.box.addLayout(self.head)
        self.box.addWidget(body, 1)
        self.set_scale(1.0)

    def set_scale(self, k):
        """枠・見出し・中の部品を、倍率 k の大きさにする."""
        self.setStyleSheet(
            f'#card {{ background: {th.CARD}; border: 1px solid {th.EDGE};'
            f' border-radius: {round(18 * k)}px; }}')
        self.box.setContentsMargins(round(18 * k), round(14 * k), round(18 * k), round(16 * k))
        self.box.setSpacing(round(10 * k))
        self.head.setSpacing(round(10 * k))
        self.title_label.setFont(th.font(11, bold=True, scale=k))
        self.tag_label.setFont(th.font(8, bold=True, spacing=2.0, scale=k))
        self.body.set_scale(k)


class _View(QWidget):
    """Snapshot を受け取って描き直す部品の共通部分."""

    BASE_MIN = (200, 200)       # 倍率 1 のときの、いちばん小さい大きさ

    def __init__(self):
        super().__init__()
        self.snap = cs.Snapshot()
        self.k = 1.0                # ウィンドウの大きさに合わせた倍率
        self.set_scale(1.0)

    def set_snapshot(self, snap):
        self.snap = snap
        self.update()

    def set_scale(self, k):
        self.k = k
        self.setMinimumSize(round(self.BASE_MIN[0] * k), round(self.BASE_MIN[1] * k))
        self.update()

    def lw(self):
        """倍率 1 のときの大きさに直した、幅."""
        return self.width() / self.k

    def lh(self):
        """倍率 1 のときの大きさに直した、高さ."""
        return self.height() / self.k

    def painter(self):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)
        p.scale(self.k, self.k)     # ここから先は、倍率 1 のときの大きさで描けばよい
        return p


class RobotView(_View):
    """ロボットを上から見た図。画面の上がロボットの前."""

    BASE_MIN = (280, 300)
    TOP = 54.0          # 上の数字(速さ・向き)の高さ
    BOTTOM = 22.0       # 下の凡例の高さ

    def __init__(self, max_linear, max_angular):
        super().__init__()
        self.max_linear = max_linear
        self.max_angular = max_angular

    def paintEvent(self, _event):
        p = self.painter()
        w, h = self.lw(), self.lh()
        # 絵は、上の数字(TOP)と下の凡例(BOTTOM)の間のまん中に描く
        side = min(w, h - self.TOP - self.BOTTOM)
        cx, cy = w / 2.0, self.TOP + (h - self.TOP - self.BOTTOM) / 2.0

        # 背景: レーダーのような目盛り
        p.setBrush(Qt.NoBrush)
        p.setPen(th.pen(th.EDGE, 1.0))
        for k in (0.47, 0.32, 0.17):
            p.drawEllipse(QPointF(cx, cy), side * k, side * k)
        p.drawLine(QPointF(cx - side * 0.47, cy), QPointF(cx + side * 0.47, cy))
        p.drawLine(QPointF(cx, cy - side * 0.47), QPointF(cx, cy + side * 0.47))

        snap = self.snap
        self.draw_readouts(p, w)
        arrows = snap.targets + snap.actuals
        if not arrows:
            p.setPen(th.color(th.DIM))
            p.setFont(th.font(10))
            p.drawText(QRectF(0, 0, w, h), Qt.AlignCenter, '足回りのデータを待っています')
            return

        half_x = max(abs(a.x) for a in arrows) or 0.1      # 前後の広がり [m]
        half_y = max(abs(a.y) for a in arrows) or 0.1      # 左右の広がり [m]
        scale = side * 0.25 / max(half_x, half_y)          # [m] → [画面の点]

        def point(x, y):
            return QPointF(cx - y * scale, cy - x * scale)

        # 車体
        margin = side * 0.075
        body = QRectF(cx - half_y * scale - margin, cy - half_x * scale - margin,
                      2 * (half_y * scale + margin), 2 * (half_x * scale + margin))
        fill = QLinearGradient(body.topLeft(), body.bottomRight())
        fill.setColorAt(0.0, th.color(th.ACCENT, 34))
        fill.setColorAt(1.0, th.color(th.ACCENT_2, 20))
        p.setBrush(fill)
        p.setPen(th.pen(th.ACCENT, 1.4, 120))
        p.drawRoundedRect(body, 24, 24)
        # 「前」の印
        p.setPen(Qt.NoPen)
        p.setBrush(th.color(th.ACCENT))
        top = QPointF(cx, body.top() - 13)
        p.drawPolygon(QPolygonF([top, QPointF(cx - 8, body.top() - 2),
                                 QPointF(cx + 8, body.top() - 2)]))
        p.setPen(th.color(th.DIM))
        p.setFont(th.font(8, bold=True, spacing=1.5))
        p.drawText(QRectF(cx + 14, body.top() - 20, 80, 18), Qt.AlignLeft | Qt.AlignVCenter, '前')

        self.draw_motion(p, cx, cy, side)

        # 各モジュール / ホイール: 目標(点線)、実際(実線)
        longest = side * 0.17

        def tip(a):
            ratio = max(0.0, min(1.0, (a.length - 0.035) / 0.2))
            length = 14.0 + ratio * longest
            start = point(a.x, a.y)
            return start, QPointF(start.x() - math.sin(a.heading) * length,
                                  start.y() - math.cos(a.heading) * length)

        for a in snap.targets:
            th.draw_arrow(p, *tip(a), th.TEXT, 1.6, head=9, alpha=150, dashed=True)
        for a in snap.actuals:
            self.draw_tire(p, point(a.x, a.y), a.heading, side)
            th.draw_arrow(p, *tip(a), th.GOOD, 3.6, head=11)
        if not snap.actuals:        # 実際の向きが届いていないときも、場所だけは描く
            for a in snap.targets:
                self.draw_tire(p, point(a.x, a.y), a.heading, side)

        self.draw_legend(p, h)

    def draw_readouts(self, p, w):
        """左上に実際の速さ、右上に向きを、大きな数字で出す."""
        actual = self.snap.actual
        speed = f'{math.hypot(actual[0], actual[1]):.2f}' if actual is not None else '--'
        heading = f'{math.degrees(self.snap.pose[2]):+.0f}°' if actual is not None else '--'
        for text, unit, label, align, x in ((speed, 'm/s', '速さ', Qt.AlignLeft, 0.0),
                                            (heading, '', '向き', Qt.AlignRight, w - 150.0)):
            p.setPen(th.color(th.FAINT))
            p.setFont(th.font(8, bold=True, spacing=1.5))
            p.drawText(QRectF(x, 0, 150, 16), align | Qt.AlignVCenter, label)
            number_font = th.font(20, bold=True)
            p.setPen(th.color(th.TEXT))
            p.setFont(number_font)
            p.drawText(QRectF(x, 14, 150, 34), align | Qt.AlignVCenter, text)
            if unit:
                p.setPen(th.color(th.DIM))
                p.setFont(th.font(9))
                offset = QFontMetricsF(number_font).horizontalAdvance(text) + 6
                p.drawText(QRectF(x + offset, 22, 60, 24), Qt.AlignLeft | Qt.AlignVCenter, unit)

    def draw_tire(self, p, center, heading, side):
        """タイヤ(向きに合わせて回した、角の丸い四角)."""
        p.save()
        p.translate(center)
        p.rotate(-math.degrees(heading))       # 画面では時計回りが正なので、符号を反転する
        width, length = side * 0.035, side * 0.085
        p.setPen(th.pen(th.FAINT, 1.2))
        p.setBrush(th.color(th.BG))
        p.drawRoundedRect(QRectF(-width / 2, -length / 2, width, length), width * 0.4,
                          width * 0.4)
        p.restore()

    def draw_motion(self, p, cx, cy, side):
        """中心に、実際に進んでいる向き(水色の矢印)と回転(紫の弧)を描く."""
        actual = self.snap.actual
        center = QPointF(cx, cy)
        if actual is not None:
            vx, vy, omega = actual
            speed = min(1.0, math.hypot(vx, vy) / self.max_linear)
            if speed > 0.02:
                length = speed * side * 0.21
                norm = math.hypot(vx, vy)
                end = QPointF(cx - vy / norm * length, cy - vx / norm * length)
                th.draw_arrow(p, center, end, th.ACCENT, 5.0, head=14, alpha=235)
            turn = max(-1.0, min(1.0, omega / self.max_angular))
            if abs(turn) > 0.02:
                radius = side * 0.07
                span = turn * 300.0                     # 度。反時計回りが正
                rect = QRectF(cx - radius, cy - radius, 2 * radius, 2 * radius)
                p.setBrush(Qt.NoBrush)
                p.setPen(th.pen(th.ACCENT_2, 3.0))
                p.drawArc(rect, 90 * 16, int(span * 16))
                end_angle = math.radians(90.0 + span)
                end = QPointF(cx + radius * math.cos(end_angle), cy - radius * math.sin(end_angle))
                sign = 1.0 if turn > 0 else -1.0
                p.setPen(Qt.NoPen)
                p.setBrush(th.color(th.ACCENT_2))
                p.drawPolygon(th.arrow_head(
                    QPointF(end.x() - sign * math.sin(end_angle) * 5,
                            end.y() - sign * math.cos(end_angle) * 5),
                    -sign * math.sin(end_angle), -sign * math.cos(end_angle), 10))
        p.setPen(Qt.NoPen)
        p.setBrush(th.color(th.TEXT))
        p.drawEllipse(center, 3.2, 3.2)

    def draw_legend(self, p, h):
        p.setFont(th.font(8))
        x, y = 2.0, h - 9.0
        for label, hex_color, dashed in (('実際', th.GOOD, False), ('目標', th.TEXT, True),
                                         ('進む向き', th.ACCENT, False), ('回転', th.ACCENT_2, False)):
            p.setPen(th.pen(hex_color, 2.4, 255 if not dashed else 150, dashed))
            p.drawLine(QPointF(x, y), QPointF(x + 16, y))
            p.setPen(th.color(th.DIM))
            width = QFontMetricsF(p.font()).horizontalAdvance(label)
            p.drawText(QRectF(x + 21, y - 9, width + 4, 18), Qt.AlignLeft | Qt.AlignVCenter, label)
            x += 21 + width + 14


class TrailView(_View):
    """走った軌跡。起動したときの前が、画面の上."""

    BASE_MIN = (240, 240)

    GRID_STEPS = (0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0)

    def paintEvent(self, _event):
        p = self.painter()
        w, h = self.lw(), self.lh()
        snap = self.snap
        x, y, yaw = snap.pose
        points = snap.trail + [(x, y)]
        xs = [q[0] for q in points] + [0.0]
        ys = [q[1] for q in points] + [0.0]
        mid_x, mid_y = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
        span = max(max(xs) - min(xs), max(ys) - min(ys), 1.6) * 1.3     # 表示する広さ [m]
        scale = (min(w, h) - 24.0) / span

        def point(px, py):
            return QPointF(w / 2.0 - (py - mid_y) * scale, (h - 22.0) / 2.0 - (px - mid_x) * scale)

        clip = QPainterPath()
        clip.addRoundedRect(QRectF(0, 0, w, h - 22), 12, 12)
        p.setPen(Qt.NoPen)
        p.setBrush(th.color(th.BG))
        p.drawPath(clip)
        p.save()
        p.setClipPath(clip)

        # マス目
        cell = next((s for s in self.GRID_STEPS if s * scale >= 30.0), self.GRID_STEPS[-1])
        reach_x, reach_y = h / scale, w / scale

        def lines(middle, reach):
            """画面に入る範囲の、マス目の線の番号(0 がスタート地点を通る線)."""
            first = math.floor((middle - reach) / cell)
            return range(first, math.ceil((middle + reach) / cell) + 1)

        for k in lines(mid_x, reach_x):
            p.setPen(th.pen(th.FAINT if k == 0 else th.EDGE, 1.0))
            p.drawLine(point(k * cell, mid_y - reach_y), point(k * cell, mid_y + reach_y))
        for k in lines(mid_y, reach_y):
            p.setPen(th.pen(th.FAINT if k == 0 else th.EDGE, 1.0))
            p.drawLine(point(mid_x - reach_x, k * cell), point(mid_x + reach_x, k * cell))

        # スタート地点
        p.setBrush(Qt.NoBrush)
        p.setPen(th.pen(th.DIM, 1.4))
        p.drawEllipse(point(0.0, 0.0), 5, 5)

        # 軌跡(古いところほど薄く)
        groups = 10
        n = len(points)
        for g in range(groups):
            first, last = n * g // groups, min(n - 1, n * (g + 1) // groups)
            if last <= first:
                continue
            path = QPainterPath(point(*points[first]))
            for q in points[first + 1:last + 1]:
                path.lineTo(point(*q))
            p.setBrush(Qt.NoBrush)
            p.setPen(th.pen(th.ACCENT, 2.6, 50 + 205 * (g + 1) // groups))
            p.drawPath(path)

        # ロボット(向きを指す三角)
        here = point(x, y)
        p.setPen(Qt.NoPen)
        p.setBrush(th.color(th.ACCENT, 46))
        p.drawEllipse(here, 17, 17)
        p.translate(here)
        p.rotate(-math.degrees(yaw))
        p.setBrush(th.color(th.TEXT))
        p.drawPolygon(QPolygonF([QPointF(0, -11), QPointF(7.5, 8), QPointF(0, 4),
                                 QPointF(-7.5, 8)]))
        p.restore()

        p.setFont(th.font(9))
        p.setPen(th.color(th.DIM))
        p.drawText(QRectF(2, h - 20, w, 20), Qt.AlignLeft | Qt.AlignVCenter,
                   f'x {x:+.2f} m    y {y:+.2f} m    向き {math.degrees(yaw):+.0f}°')
        p.setPen(th.color(th.FAINT))
        p.drawText(QRectF(0, h - 20, w - 2, 20), Qt.AlignRight | Qt.AlignVCenter,
                   f'1 マス {cell:g} m')


class SpeedGauges(_View):
    """前後・左右・回転の、指令の速さ(白い印)と実際の速さ(バーと数字)."""

    BASE_MIN = (260, 180)
    MAX_ROW_H = 96.0

    def __init__(self, max_linear, max_angular):
        super().__init__()
        self.rows = (('前後', 'm/s', max_linear), ('左右', 'm/s', max_linear),
                     ('回転', 'rad/s', max_angular))

    def paintEvent(self, _event):
        p = self.painter()
        w, h = self.lw(), self.lh()
        # 縦に長いときは、行の間を広げすぎずに、まとめてまん中に置く
        row_h = min(h / len(self.rows), self.MAX_ROW_H)
        top = (h - row_h * len(self.rows)) / 2.0
        snap = self.snap
        for i, (label, unit, limit) in enumerate(self.rows):
            mid = top + row_h * (i + 0.5)
            command = snap.command[i]
            actual = snap.actual[i] if snap.actual is not None else None
            slow = actual is not None and abs(command) > 0.05 and abs(actual) < 0.8 * abs(command)

            # 上: 名前と単位(左)、指令の速さ(小さく)と実際の速さ(大きく)(右)
            p.setPen(th.color(th.TEXT))
            p.setFont(th.font(11, bold=True))
            p.drawText(QRectF(0, mid - 24, 44, 26), Qt.AlignLeft | Qt.AlignVCenter, label)
            p.setPen(th.color(th.FAINT))
            p.setFont(th.font(8))
            p.drawText(QRectF(44, mid - 22, 60, 26), Qt.AlignLeft | Qt.AlignVCenter, unit)
            number_font = th.font(17, bold=True)
            number = f'{actual:+.2f}' if actual is not None else '--'
            p.setPen(th.color(th.WARN if slow else th.TEXT))
            p.setFont(number_font)
            p.drawText(QRectF(w - 130, mid - 28, 130, 30), Qt.AlignRight | Qt.AlignVCenter, number)
            p.setPen(th.color(th.DIM))
            p.setFont(th.font(9))
            number_w = QFontMetricsF(number_font).horizontalAdvance('+0.00') + 12
            p.drawText(QRectF(0, mid - 22, w - number_w, 26), Qt.AlignRight | Qt.AlignVCenter,
                       f'指令 {command:+.2f}')
            # 下: バー(中心が 0)。色の付いた部分が実際、白い印が指令。頭打ちのときはオレンジ
            center, half = w / 2.0, w / 2.0
            bar_y = mid + 14
            p.setPen(Qt.NoPen)
            p.setBrush(th.color(th.TRACK))
            p.drawRoundedRect(QRectF(0, bar_y - 4, w, 8), 4, 4)
            if actual is not None:
                reach = max(-1.0, min(1.0, actual / limit)) * half
                if abs(reach) > 0.5:
                    fill = QLinearGradient(QPointF(center, 0), QPointF(center + reach, 0))
                    fill.setColorAt(0.0, th.color(th.WARN if slow else th.ACCENT_2))
                    fill.setColorAt(1.0, th.color(th.WARN if slow else th.ACCENT))
                    p.setBrush(fill)
                    start = min(center, center + reach)
                    p.drawRoundedRect(QRectF(start, bar_y - 4, abs(reach), 8), 4, 4)
            p.setBrush(th.color(th.FAINT))
            p.drawRect(QRectF(center - 0.5, bar_y - 8, 1, 16))
            mark = center + max(-1.0, min(1.0, command / limit)) * (half - 2)
            p.setBrush(th.color(th.TEXT))
            p.drawRoundedRect(QRectF(mark - 1.5, bar_y - 9, 3, 18), 1.5, 1.5)


class KeyPad(_View):
    """WASD のキー。押しているキーが光る。マウスで押してもよい."""

    BASE_MIN = (250, 170)

    keyChanged = pyqtSignal(str, bool)     # (キーの名前, 押した / 離した)

    # (キーの名前, 表示, 動き, 行, 列)
    KEYS = (('q', 'Q', '左回り', 0, 0), ('w', 'W', '前', 0, 1), ('e', 'E', '右回り', 0, 2),
            ('a', 'A', '左', 1, 0), ('s', 'S', '後ろ', 1, 1), ('d', 'D', '右', 1, 2))
    FOOTER = 26.0

    def __init__(self, hint=''):
        super().__init__()
        self.active = set()         # 押しているキー('w' など。Shift は 'shift')
        self.focused = False        # このウィンドウが選ばれているか
        self.stopped = False        # 停止ボタンで止まっているか
        self.hint = hint            # 右下に出す、速さの説明
        self.mouse_key = None

    def set_keys(self, active, focused, stopped):
        self.active = set(active)
        self.focused = focused
        self.stopped = stopped
        self.update()

    def rects(self):
        """キーの名前 → 画面上の四角."""
        w, h = self.lw(), self.lh() - self.FOOTER
        gap = 8.0
        key = min((w - 2 * gap) / 3.0, (h - 2 * gap) / 2.7, 78.0)
        x0 = (w - (3 * key + 2 * gap)) / 2.0
        y0 = (h - (2.7 * key + 2 * gap)) / 2.0
        result = {name: QRectF(x0 + col * (key + gap), y0 + row * (key + gap), key, key)
                  for name, _, _, row, col in self.KEYS}
        result['shift'] = QRectF(x0, y0 + 2 * (key + gap), 3 * key + 2 * gap, key * 0.7)
        return result

    def paintEvent(self, _event):
        p = self.painter()
        rects = self.rects()
        labels = {name: (cap, action) for name, cap, action, _, _ in self.KEYS}
        labels['shift'] = ('Shift', '速く')
        for name, rect in rects.items():
            on = name in self.active
            if on:
                fill = QLinearGradient(rect.topLeft(), rect.bottomRight())
                fill.setColorAt(0.0, th.color(th.ACCENT))
                fill.setColorAt(1.0, th.color(th.ACCENT_2))
                p.setBrush(fill)
                p.setPen(Qt.NoPen)
            else:
                p.setBrush(th.color(th.TRACK))
                p.setPen(th.pen(th.EDGE, 1.0))
            p.drawRoundedRect(rect, 12, 12)
            cap, action = labels[name]
            if name == 'shift':
                p.setPen(th.color('#06121a' if on else th.TEXT))
                p.setFont(th.font(11, bold=True))
                p.drawText(rect, Qt.AlignCenter, f'{cap}  {action}')
                continue
            p.setPen(th.color('#06121a' if on else th.TEXT))
            p.setFont(th.font(15, bold=True))
            p.drawText(QRectF(rect.left(), rect.top() + rect.height() * 0.10, rect.width(),
                              rect.height() * 0.5), Qt.AlignCenter, cap)
            p.setPen(th.color('#06121a' if on else th.DIM))
            p.setFont(th.font(8))
            p.drawText(QRectF(rect.left(), rect.top() + rect.height() * 0.58, rect.width(),
                              rect.height() * 0.32), Qt.AlignCenter, action)

        y = self.lh() - self.FOOTER / 2.0 + 2.0
        if self.stopped:
            dot, ink, text = th.WARN, th.WARN, '停止中(「再開」を押すまで動きません)'
        elif self.focused:
            dot, ink, text = th.GOOD, th.TEXT, 'キーボードで操作できます'
        else:
            dot, ink, text = th.FAINT, th.DIM, 'クリックすると操作できます'
        p.setPen(Qt.NoPen)
        p.setBrush(th.color(dot))
        p.drawEllipse(QPointF(6, y), 4, 4)
        p.setPen(th.color(ink))
        p.setFont(th.font(9))
        p.drawText(QRectF(16, y - 10, self.lw(), 20), Qt.AlignLeft | Qt.AlignVCenter, text)
        metrics = QFontMetricsF(p.font())
        if 16 + metrics.horizontalAdvance(text) + 16 + metrics.horizontalAdvance(self.hint) < \
                self.lw():      # 入るときだけ、右に速さの説明を出す
            p.setPen(th.color(th.FAINT))
            p.drawText(QRectF(0, y - 10, self.lw() - 2, 20), Qt.AlignRight | Qt.AlignVCenter,
                       self.hint)

    def mousePressEvent(self, event):
        pos = QPointF(event.pos()) / self.k     # 倍率 1 のときの位置に直す
        for name, rect in self.rects().items():
            if rect.contains(pos):
                self.mouse_key = name
                self.keyChanged.emit(name, True)
                return

    def mouseReleaseEvent(self, _event):
        if self.mouse_key is not None:
            self.keyChanged.emit(self.mouse_key, False)
            self.mouse_key = None


class StatusList(_View):
    """状態の一覧(/diagnostics)。広ければ 2 列に並べる."""

    ROW_H = 28.0

    def set_scale(self, k):
        self.k = k
        self.setFixedHeight(round(self.ROW_H * 4 * k))
        self.update()

    def paintEvent(self, _event):
        p = self.painter()
        w, h = self.lw(), self.lh()
        rows = self.snap.rows
        if not rows:
            p.setPen(th.color(th.DIM))
            p.setFont(th.font(10))
            p.drawText(QRectF(0, 0, w, h), Qt.AlignCenter,
                       'ロボットからの知らせ(/diagnostics)を待っています')
            return
        columns = 2 if w >= 720 and len(rows) > 4 else 1
        per_column = math.ceil(len(rows) / columns)
        column_w = (w - (columns - 1) * 28.0) / columns
        for i, row in enumerate(rows):
            x = (i // per_column) * (column_w + 28.0)
            y = (i % per_column) * self.ROW_H
            if y + self.ROW_H > h + 1:
                continue                    # 入りきらない分は描かない
            self.draw_row(p, row, x, y, column_w)

    def draw_row(self, p, row, x, y, width):
        mid = y + self.ROW_H / 2.0
        hex_color = th.LEVEL_COLOR[row.level]
        p.setPen(Qt.NoPen)
        p.setBrush(th.color(hex_color, 60))
        p.drawEllipse(QPointF(x + 8, mid), 7, 7)
        p.setBrush(th.color(hex_color))
        p.drawEllipse(QPointF(x + 8, mid), 3.6, 3.6)

        name_font, text_font, values_font = th.font(10, bold=True), th.font(10), th.font(9)
        name_w = min(QFontMetricsF(name_font).horizontalAdvance(row.name) + 14.0, width * 0.42)
        message = row.message
        if row.level != cs.OK:
            message = f'{cs.LEVEL_TEXT[row.level]}: {message}'
        rest = max(0.0, width - 24 - name_w)
        # 文を先に入れて、残ったところに補足を入れる(入りきらなければ切り詰める)
        message_w = min(QFontMetricsF(text_font).horizontalAdvance(message) + 12.0, rest)
        values_w = max(0.0, rest - message_w)
        p.setPen(th.color(th.TEXT))
        p.setFont(name_font)
        p.drawText(QRectF(x + 24, y, name_w, self.ROW_H), Qt.AlignLeft | Qt.AlignVCenter,
                   QFontMetricsF(name_font).elidedText(row.name, Qt.ElideRight, name_w))
        p.setPen(th.color(th.DIM if row.level == cs.OK else hex_color))
        p.setFont(text_font)
        p.drawText(QRectF(x + 24 + name_w, y, message_w, self.ROW_H),
                   Qt.AlignLeft | Qt.AlignVCenter,
                   QFontMetricsF(text_font).elidedText(message, Qt.ElideRight, message_w))
        if row.values and values_w > 40:
            p.setPen(th.color(th.FAINT))
            p.setFont(values_font)
            p.drawText(QRectF(x + width - values_w, y, values_w, self.ROW_H),
                       Qt.AlignRight | Qt.AlignVCenter,
                       QFontMetricsF(values_font).elidedText(row.values, Qt.ElideRight, values_w))
