"""
ロボットコンソールの画面の部品(PyQt5).

    RobotView   … ロボットを上から見た図(各モジュール / ホイールの矢印、進む向き、回転)
    TrailView   … 地図の上の、ロボットの位置・走った軌跡・LiDAR の点
    LidarView   … LiDAR が見ている物(ロボットが中心)
    TimePlot    … 指令と実際の値の、時間のグラフ
    StmTable    … STM32 ごとの通信の様子(指令・状態の中身と回数)
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
from PyQt5.QtGui import (QFontMetricsF, QImage, QLinearGradient, QPainter, QPainterPath,
                         QPolygonF, qRgb, QTransform)
from PyQt5.QtWidgets import QAbstractItemView, QHeaderView, QTableWidget, QTableWidgetItem, QWidget

from . import console_state as cs
from . import console_theme as th


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
        p.fillRect(self.rect(), th.color(th.CARD))      # 図の部分の背景(エディタの図と同じ色)
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
    """
    地図の上の、ロボットの位置・走った軌跡・LiDAR の点(地図の座標。地図の +x が画面の上).

    地図(/map)が届いていれば、地図全体が入るように表示する。
    """

    BASE_MIN = (240, 240)
    # 位置と向きを選ぶもの: ロボットを置く(シミュレーター)/ 自動操縦のゴール / AMCL に位置を知らせる
    POSE_MODES = ('pose', 'goal', 'initial')

    GRID_STEPS = (0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0)
    FREE_RGB = (255, 255, 255)      # 地図の通れるマス
    BLOCKED_RGB = (138, 148, 165)   # 地図のふさがっているマス
    UNKNOWN_RGB = (228, 233, 240)   # 地図の分からないマス

    # 地図をクリック(+ ドラッグで向き)した (選ぶもの, x, y, 向き)。選ぶものは POSE_MODES のどれか
    posePicked = pyqtSignal(str, float, float, float)
    pointPicked = pyqtSignal(float, float)          # 地図をクリックした (x, y)

    def __init__(self):
        super().__init__()
        self.show_map = True
        self.show_scan = True
        self.map_image = None
        self.map_version = None
        self.obstacles = []         # シミュレーターで置いた障害物 [(x, y, 半径)]
        self.pick_mode = None       # None / POSE_MODES のどれか / 'point'(障害物を置く)
        self.click_mode = None      # ボタンを押さなくても、クリックだけで選ぶもの(自動操縦のタブではゴール)
        self.pick_start = None      # ドラッグを始めた地図の座標
        self.pick_now = None
        self.view = None            # 最後に描いたときの (mid_x, mid_y, scale)。クリックの位置を直すのに使う

    def set_pick_mode(self, mode):
        """次のクリックで選ぶもの(1 回だけ。終わると click_mode にもどる)."""
        self.pick_mode = mode
        self.pick_start = self.pick_now = None
        self.setCursor(Qt.CrossCursor if mode or self.click_mode else Qt.ArrowCursor)
        self.update()

    def set_click_mode(self, mode):
        """ボタンを押さなくても、クリックするたびに選ぶもの(None でやめる)."""
        if mode != self.click_mode:
            self.click_mode = mode
            if self.pick_start is None:
                self.set_pick_mode(self.pick_mode)      # カーソルを変える

    def to_map(self, pos):
        """画面の点 → 地図の座標."""
        if self.view is None:
            return None
        mid_x, mid_y, scale = self.view
        sx, sy = pos.x() / self.k, pos.y() / self.k
        return (mid_x - (sy - (self.lh() - 22.0) / 2.0) / scale,
                mid_y - (sx - self.lw() / 2.0) / scale)

    def mousePressEvent(self, event):
        if not self.pick_mode and self.click_mode:
            self.pick_mode = self.click_mode
        if self.pick_mode and event.button() == Qt.LeftButton:
            self.pick_start = self.pick_now = self.to_map(event.pos())
            self.update()

    def mouseMoveEvent(self, event):
        if self.pick_start is not None:
            self.pick_now = self.to_map(event.pos())
            self.update()

    def mouseReleaseEvent(self, event):
        if self.pick_start is None or event.button() != Qt.LeftButton:
            return
        (x0, y0), (x1, y1) = self.pick_start, self.to_map(event.pos())
        mode = self.pick_mode
        self.set_pick_mode(None)
        if mode in self.POSE_MODES:
            # ドラッグした向きがロボットの向き。ほぼ動かしていなければ、今の向きのまま
            yaw = math.atan2(y1 - y0, x1 - x0) if math.hypot(x1 - x0, y1 - y0) > 0.05 \
                else self.snap.map_pose[2]
            self.posePicked.emit(mode, x0, y0, yaw)
        elif mode == 'point':
            self.pointPicked.emit(x0, y0)

    def image_of(self, grid):
        """地図を絵にする(届くたびに 1 回だけ)。画像の上の行が地図の +y の端."""
        if grid.version != self.map_version:
            image = QImage(grid.width, grid.height, QImage.Format_RGB32)
            colors = {0: qRgb(*self.FREE_RGB), 255: qRgb(*self.UNKNOWN_RGB)}
            blocked = qRgb(*self.BLOCKED_RGB)
            for r in range(grid.height):
                row = grid.cells[r * grid.width:(r + 1) * grid.width]
                v = grid.height - 1 - r
                for c, value in enumerate(row):
                    image.setPixel(c, v, colors.get(value, blocked if 50 <= value <= 100
                                                    else qRgb(*self.FREE_RGB)))
            self.map_image, self.map_version = image, grid.version
        return self.map_image

    def paintEvent(self, _event):
        p = self.painter()
        w, h = self.lw(), self.lh()
        snap = self.snap
        x, y, yaw = snap.map_pose
        trail = snap.map_trail()
        points = trail + [(x, y), (0.0, 0.0)]
        grid = snap.grid if self.show_map else None
        if grid is not None:
            x0, x1, y0, y1 = grid.bounds
            points += [(x0, y0), (x1, y1)]
        xs = [q[0] for q in points]
        ys = [q[1] for q in points]
        mid_x, mid_y = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
        span_x = max(max(xs) - min(xs), 1.6) * (1.08 if grid else 1.3)    # 表示する広さ [m]
        span_y = max(max(ys) - min(ys), 1.6) * (1.08 if grid else 1.3)
        scale = min((h - 34.0) / span_x, (w - 12.0) / span_y)

        def point(px, py):
            return QPointF(w / 2.0 - (py - mid_y) * scale, (h - 22.0) / 2.0 - (px - mid_x) * scale)

        self.view = (mid_x, mid_y, scale)
        clip = QPainterPath()
        clip.addRoundedRect(QRectF(0, 0, w, h - 22), 12, 12)
        p.setPen(Qt.NoPen)
        p.setBrush(th.color(th.BG))
        p.drawPath(clip)
        p.save()
        p.setClipPath(clip)

        # 地図: 画像の画素 (u, v) → 地図の (x, y) → 画面
        if grid is not None:
            image = self.image_of(grid)
            res = grid.resolution
            p.save()
            p.setRenderHint(QPainter.SmoothPixmapTransform, False)
            p.setTransform(QTransform(
                0.0, -res * scale, res * scale, 0.0,
                w / 2.0 - (grid.origin_y + grid.height * res - mid_y) * scale,
                (h - 22.0) / 2.0 - (grid.origin_x - mid_x) * scale), True)
            p.drawImage(0, 0, image)
            p.restore()

        # マス目
        cell = next((s for s in self.GRID_STEPS if s * scale >= 30.0), self.GRID_STEPS[-1])
        reach_x, reach_y = h / scale, w / scale

        def lines(middle, reach):
            """画面に入る範囲の、マス目の線の番号(0 が地図の原点を通る線)."""
            first = math.floor((middle - reach) / cell)
            return range(first, math.ceil((middle + reach) / cell) + 1)

        for k in lines(mid_x, reach_x):
            p.setPen(th.pen(th.FAINT if k == 0 else th.EDGE, 1.0, 160 if grid else 255))
            p.drawLine(point(k * cell, mid_y - reach_y), point(k * cell, mid_y + reach_y))
        for k in lines(mid_y, reach_y):
            p.setPen(th.pen(th.FAINT if k == 0 else th.EDGE, 1.0, 160 if grid else 255))
            p.drawLine(point(mid_x - reach_x, k * cell), point(mid_x + reach_x, k * cell))

        # スタート地点(地図の原点)
        p.setBrush(Qt.NoBrush)
        p.setPen(th.pen(th.DIM, 1.4))
        p.drawEllipse(point(0.0, 0.0), 5, 5)

        # 軌跡(古いところほど薄く)
        groups = 10
        n = len(trail)
        for g in range(groups):
            first, last = n * g // groups, min(n - 1, n * (g + 1) // groups)
            if last <= first:
                continue
            path = QPainterPath(point(*trail[first]))
            for q in trail[first + 1:last + 1]:
                path.lineTo(point(*q))
            p.setBrush(Qt.NoBrush)
            p.setPen(th.pen(th.ACCENT, 2.6, 50 + 205 * (g + 1) // groups))
            p.drawPath(path)

        # シミュレーターで置いた障害物
        for ox, oy, r in self.obstacles:
            p.setPen(th.pen(th.WARN, 1.6))
            p.setBrush(th.color(th.WARN, 60))
            p.drawEllipse(point(ox, oy), r * scale, r * scale)

        # 自動操縦の道(紫の線)とゴール(紫の旗の矢印)
        if len(snap.plan) >= 2:
            path = QPainterPath(point(*snap.plan[0]))
            for q in snap.plan[1:]:
                path.lineTo(point(*q))
            p.setBrush(Qt.NoBrush)
            p.setPen(th.pen(th.ACCENT_2, 3.0))
            p.drawPath(path)
        if snap.goal is not None:
            gx, gy, gyaw = snap.goal
            alpha = 255 if snap.navigating else 110
            start = point(gx, gy)
            p.setPen(th.pen(th.ACCENT_2, 2.0, alpha))
            p.setBrush(th.color(th.ACCENT_2, alpha // 4))
            p.drawEllipse(start, 10, 10)
            th.draw_arrow(p, start, point(gx + 0.35 * math.cos(gyaw), gy + 0.35 * math.sin(gyaw)),
                          th.ACCENT_2, 2.4, head=10, alpha=alpha)

        # シミュレーションの本当の位置(灰色の三角。オドメトリとずれると離れて見える)
        if snap.truth is not None:
            tx, ty, tyaw = snap.truth
            p.save()
            p.translate(point(tx, ty))
            p.rotate(-math.degrees(tyaw))
            p.setPen(th.pen(th.DIM, 1.6))
            p.setBrush(Qt.NoBrush)
            p.drawPolygon(QPolygonF([QPointF(0, -13), QPointF(9, 10), QPointF(0, 5),
                                     QPointF(-9, 10)]))
            p.restore()

        # LiDAR の点(ロボットから見た位置 → 地図の座標)
        if self.show_scan and snap.scan:
            p.setPen(Qt.NoPen)
            p.setBrush(th.color(th.BAD, 210))
            for sx, sy in snap.scan:
                mx, my = cs.compose((x, y, yaw), sx, sy)
                p.drawEllipse(point(mx, my), 2.2, 2.2)

        # 置こうとしている位置と向き(ドラッグ中)
        if self.pick_start is not None and self.pick_now is not None:
            start, now_pt = point(*self.pick_start), point(*self.pick_now)
            p.setPen(th.pen(th.ACCENT_2, 2.0, dashed=True))
            p.setBrush(th.color(th.ACCENT_2, 50))
            p.drawEllipse(start, 14, 14)
            if self.pick_mode in self.POSE_MODES:
                th.draw_arrow(p, start, now_pt, th.ACCENT_2, 2.4, head=12)

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
        notes = ['地図あり' if snap.grid else '地図なし', f'LiDAR {len(snap.scan)} 点' if snap.scan
                 else 'LiDAR なし', f'1 マス {cell:g} m']
        p.drawText(QRectF(0, h - 20, w - 2, 20), Qt.AlignRight | Qt.AlignVCenter,
                   '    '.join(notes))


class LidarView(_View):
    """LiDAR が見ている物(ロボットが中心、画面の上がロボットの前)."""

    BASE_MIN = (240, 240)
    RING_STEPS = (0.25, 0.5, 1.0, 2.0, 5.0)

    def paintEvent(self, _event):
        p = self.painter()
        w, h = self.lw(), self.lh()
        scan = self.snap.scan
        cx, cy = w / 2.0, (h - 22.0) / 2.0
        reach = max([math.hypot(px, py) for px, py in scan] + [1.0]) * 1.08
        reach = min(reach, 12.0)
        scale = (min(w, h - 22.0) / 2.0 - 8.0) / reach
        ring = next((s for s in self.RING_STEPS if reach / s <= 6), self.RING_STEPS[-1])

        def point(px, py):
            return QPointF(cx - py * scale, cy - px * scale)

        p.setBrush(Qt.NoBrush)
        k = 1
        while k * ring <= reach:
            p.setPen(th.pen(th.EDGE, 1.0))
            p.drawEllipse(QPointF(cx, cy), k * ring * scale, k * ring * scale)
            k += 1
        p.setPen(th.pen(th.EDGE, 1.0))
        p.drawLine(QPointF(cx, cy - reach * scale), QPointF(cx, cy + reach * scale))
        p.drawLine(QPointF(cx - reach * scale, cy), QPointF(cx + reach * scale, cy))
        if not scan:
            p.setPen(th.color(th.DIM))
            p.setFont(th.font(10))
            p.drawText(QRectF(0, 0, w, h - 22), Qt.AlignCenter,
                       'LiDAR(/scan)のデータを待っています\n(シミュレーションでは fake_lidar が出す)')
        else:
            p.setPen(Qt.NoPen)
            p.setBrush(th.color(th.BAD, 220))
            for px, py in scan:
                p.drawEllipse(point(px, py), 2.4, 2.4)
        # ロボット(上が前)
        p.setPen(Qt.NoPen)
        p.setBrush(th.color(th.ACCENT))
        p.drawPolygon(QPolygonF([QPointF(cx, cy - 12), QPointF(cx + 8, cy + 8),
                                 QPointF(cx, cy + 3), QPointF(cx - 8, cy + 8)]))
        p.setFont(th.font(9))
        p.setPen(th.color(th.DIM))
        p.drawText(QRectF(2, h - 20, w, 20), Qt.AlignLeft | Qt.AlignVCenter,
                   f'{len(scan)} 点    上がロボットの前')
        p.setPen(th.color(th.FAINT))
        p.drawText(QRectF(0, h - 20, w - 2, 20), Qt.AlignRight | Qt.AlignVCenter,
                   f'円の間隔 {ring:g} m')


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
                p.setPen(th.color('#ffffff' if on else th.TEXT))
                p.setFont(th.font(11, bold=True))
                p.drawText(rect, Qt.AlignCenter, f'{cap}  {action}')
                continue
            p.setPen(th.color('#ffffff' if on else th.TEXT))
            p.setFont(th.font(15, bold=True))
            p.drawText(QRectF(rect.left(), rect.top() + rect.height() * 0.10, rect.width(),
                              rect.height() * 0.5), Qt.AlignCenter, cap)
            p.setPen(th.color('#ffffff' if on else th.DIM))
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


class TimePlot(_View):
    """
    指令(点線)と実際(実線)の値の、時間のグラフ(右端が今).

    set_series(題名, 単位, [(時刻, 指令, 実際)]) で中身を渡す。値が None のところは描かない。
    """

    BASE_MIN = (260, 180)
    WINDOW_S = 20.0

    def __init__(self):
        super().__init__()
        self.title, self.unit, self.points = '', '', []

    def set_series(self, title, unit, points):
        self.title, self.unit, self.points = title, unit, points
        self.update()

    def paintEvent(self, _event):
        p = self.painter()
        w, h = self.lw(), self.lh()
        left, top, right, bottom = 54.0, 46.0, 12.0, 26.0
        pw, ph = w - left - right, h - top - bottom
        values = [v for _, a, b in self.points for v in (a, b) if v is not None]
        span = max([abs(v) for v in values] + [0.1]) * 1.15
        t_end = self.points[-1][0] if self.points else 0.0

        def point(t, v):
            return QPointF(left + (1.0 - (t_end - t) / self.WINDOW_S) * pw,
                           top + ph / 2.0 - v / span * ph / 2.0)

        p.setFont(th.font(10, bold=True))
        p.setPen(th.color(th.TEXT))
        p.drawText(QRectF(left, 2, pw, 24), Qt.AlignLeft | Qt.AlignVCenter,
                   f'{self.title}  [{self.unit}]')
        p.setFont(th.font(8))
        for frac in (-1.0, -0.5, 0.0, 0.5, 1.0):
            y = top + ph / 2.0 - frac * ph / 2.0
            p.setPen(th.pen(th.FAINT if frac == 0 else th.EDGE, 1.0))
            p.drawLine(QPointF(left, y), QPointF(left + pw, y))
            p.setPen(th.color(th.DIM))
            p.drawText(QRectF(0, y - 9, left - 6, 18), Qt.AlignRight | Qt.AlignVCenter,
                       f'{frac * span:+.2f}')
        for sec in range(0, int(self.WINDOW_S) + 1, 5):
            x = left + (1.0 - sec / self.WINDOW_S) * pw
            p.setPen(th.pen(th.EDGE, 1.0))
            p.drawLine(QPointF(x, top), QPointF(x, top + ph))
            p.setPen(th.color(th.DIM))
            p.drawText(QRectF(x - 30, top + ph + 4, 60, 18), Qt.AlignCenter,
                       '今' if sec == 0 else f'-{sec} 秒')
        if not self.points:
            p.setPen(th.color(th.DIM))
            p.setFont(th.font(10))
            p.drawText(QRectF(left, top, pw, ph), Qt.AlignCenter, 'データを待っています')
            return
        for index, hex_color, dashed in ((1, th.TEXT, True), (2, th.ACCENT, False)):
            path, drawing = QPainterPath(), False
            for sample in self.points:
                v = sample[index]
                if v is None or t_end - sample[0] > self.WINDOW_S:
                    drawing = False
                    continue
                q = point(sample[0], v)
                if drawing:
                    path.lineTo(q)
                else:
                    path.moveTo(q)
                    drawing = True
            p.setBrush(Qt.NoBrush)
            p.setPen(th.pen(hex_color, 1.6 if dashed else 2.2, 200 if dashed else 255, dashed))
            p.drawPath(path)
        last = self.points[-1]
        p.setFont(th.font(9))
        for i, (label, hex_color, value) in enumerate((('指令(点線)', th.DIM, last[1]),
                                                      ('実際(実線)', th.ACCENT, last[2]))):
            text = f'{label} {value:+.3f}' if value is not None else f'{label} --'
            p.setPen(th.color(hex_color))
            p.drawText(QRectF(left + i * 170, 22, 165, 20), Qt.AlignLeft | Qt.AlignVCenter, text)


class StmTable(QTableWidget):
    """STM32 ごとの通信の様子(1 行が STM32 1 枚)."""

    HEADERS = ('STM32', '指令 Pi → STM32', '状態 STM32 → Pi', '回/秒\n指令 / 状態', '最後に\n届いてから')

    def __init__(self):
        super().__init__(0, len(self.HEADERS))
        self.setHorizontalHeaderLabels(self.HEADERS)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.setSelectionMode(QAbstractItemView.NoSelection)
        self.setFocusPolicy(Qt.NoFocus)
        header = self.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.Stretch)

    @staticmethod
    def values_text(values):
        return ', '.join(f'{v:+.2f}' for v in values) if values else '--'

    def set_snapshot(self, snap):
        rows = snap.stm_rows
        if self.rowCount() != len(rows):
            self.setRowCount(len(rows))
        for r, row in enumerate(rows):
            ages = [a for a in (row.command_age, row.state_age) if a is not None]
            age = min(ages) if ages else None
            late = row.state_age is None or row.state_age > 0.5
            cells = (f'stm{row.dev_id}', self.values_text(row.command),
                     self.values_text(row.state),
                     f'{row.command_rate:.0f} / {row.state_rate:.0f}',
                     '--' if age is None else f'{age * 1000:.0f} ms')
            for c, text in enumerate(cells):
                item = self.item(r, c)
                if item is None:
                    item = QTableWidgetItem()
                    self.setItem(r, c, item)
                item.setText(text)
                item.setForeground(th.color(th.BAD if late and c in (2, 3) else th.TEXT))
