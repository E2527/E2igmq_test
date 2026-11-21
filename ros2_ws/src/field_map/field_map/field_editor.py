"""
フィールドエディタ: フィールドの形(fields/*.yaml)を、画面で描いて編集する(PC 用).

    ros2 run field_map field_editor              # fields/example.yaml を開く
    ros2 run field_map field_editor my_field     # fields/my_field.yaml(なければ新しく作る)

できること:
    ・図形(箱・円・多角形)を描く、ドラッグで動かす、角の点で大きさや形を変える、丸い点で回す
    ・右の欄で、名前・位置・大きさ・回転・高さを数字で直す
    ・フィールドの図(画像)を下に敷いて、上からなぞる(2 つの点で縮尺と位置を合わせる)
    ・画像の黒いところ(または選んだ色)から、図形を自動で作る
    ・保存すると YAML に書き出す。field_map_node が動いていれば、地図(/map)もすぐ変わる

色の意味: 青 = 通れない・LiDAR に映る、オレンジ(斜線)= 通れないが LiDAR に映らない、
          紫 = LiDAR に映るが下をくぐれる、灰色の点線 = 通れる(地図に入らない)

画面の部分だけで、計算は field_io.py と field.py にある(テストはそちら)。
"""

import copy
import json
import math
import os
import sys

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import (QBrush, QColor, QImage, QKeySequence, QPainter, QPainterPath, QPen,
                         QPixmap, QPolygonF, QTransform)
from PyQt5.QtWidgets import (QAction, QActionGroup, QApplication, QCheckBox, QColorDialog,
                             QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
                             QGraphicsDropShadowEffect, QGraphicsItem,
                             QGraphicsPixmapItem, QGraphicsScene,
                             QGraphicsSimpleTextItem, QGraphicsView, QHBoxLayout, QInputDialog,
                             QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton, QSlider,
                             QSpinBox, QTabWidget, QToolBar, QVBoxLayout, QWidget)
import yaml

from . import field as fm
from . import field_io as fio

# 図の色(ロボットコンソールの図と同じ。swerve_drive/console_theme.py)
CANVAS = QColor('#f6f8fb')      # 図の背景
GRID = QColor('#d9dfe8')        # 目盛りの線
AXIS = QColor('#9aa3b2')        # x = 0、y = 0 の線
INK = QColor('#1f2937')         # 文字、フィールドの範囲の線
ACCENT = QColor('#0b84d8')
COLORS = {    # (塗り, ふち)
    'both': (QColor(11, 132, 216, 90), QColor('#0b84d8')),
    'nav': (QColor(217, 119, 6, 70), QColor('#d97706')),
    'lidar': (QColor(109, 91, 216, 70), QColor('#6d5bd8')),
    'none': (QColor(154, 163, 178, 40), QColor('#9aa3b2')),
}
HANDLE_PX = 7           # 角の点の大きさ [画面の点]
SNAP_STEPS = (('吸着なし', 0.0), ('1 cm', 0.01), ('5 cm', 0.05), ('10 cm', 0.1))


def scene_point(x, y):
    """フィールドの座標 [m] → 画面(シーン)の座標。シーンは y が下向きなので反転する."""
    return QPointF(x, -y)


def field_point(p):
    return p.x(), -p.y()


def box_corners(shape):
    """箱の 4 つの角 [(x, y)]."""
    hx, hy = shape['size_x'] / 2, shape['size_y'] / 2
    yaw = math.radians(shape.get('yaw_deg', 0.0))
    c, s = math.cos(yaw), math.sin(yaw)
    return [(shape['x'] + c * dx - s * dy, shape['y'] + s * dx + c * dy)
            for dx, dy in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]


def to_field_shape(shape):
    """エディタの図形の辞書 → field.py の Shape(色分けの判定に使う)."""
    item = dict(shape)
    item.setdefault('name', '')
    try:
        return fm._parse_shape(item)
    except (KeyError, TypeError, ValueError):
        return None


# ---------------------------------------------------------------- 図形の絵

class ShapeItem(QGraphicsItem):
    """図形 1 つ。座標はシーンにそのまま描き、ドラッグで動いた分は離したときに図形に足す."""

    def __init__(self, editor, index):
        super().__init__()
        self.editor, self.index = editor, index
        self.drag = None            # 角の点をつかんでいるとき: ('corner' | 'rotate' | 'point', 番号)
        self.setFlags(QGraphicsItem.ItemIsSelectable | QGraphicsItem.ItemIsMovable)
        self.setAcceptHoverEvents(True)
        self.label = QGraphicsSimpleTextItem(self)
        self.label.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        self.label.setBrush(INK)
        halo = QGraphicsDropShadowEffect()                  # 画像の上でも読めるように、白いふち
        halo.setBlurRadius(6)
        halo.setOffset(0, 0)
        halo.setColor(QColor(255, 255, 255))
        self.label.setGraphicsEffect(halo)
        self.update_label()

    @property
    def shape_data(self):
        return self.editor.shapes[self.index]

    def update_label(self):
        data = self.shape_data
        self.label.setText(data.get('name', ''))
        cx, cy = self.center()
        self.label.setPos(scene_point(cx, cy))

    def center(self):
        d = self.shape_data
        if d['type'] == 'polygon':
            xs, ys = zip(*d['points'])
            return sum(xs) / len(xs), sum(ys) / len(ys)
        return d['x'], d['y']

    def outline(self):
        d = self.shape_data
        path = QPainterPath()
        if d['type'] == 'circle':
            r = d['radius']
            path.addEllipse(scene_point(d['x'], d['y']), r, r)
        else:
            pts = box_corners(d) if d['type'] == 'box' else d['points']
            path.addPolygon(QPolygonF([scene_point(x, y) for x, y in pts]))
            path.closeSubpath()
        return path

    def handles(self):
        """つかめる点 [(種類, 番号, (x, y))]."""
        d = self.shape_data
        if d['type'] == 'box':
            corners = box_corners(d)
            yaw = math.radians(d.get('yaw_deg', 0.0))
            reach = d['size_x'] / 2 + max(0.08, 20 / self.editor.view.scale_px())
            rot = (d['x'] + math.cos(yaw) * reach, d['y'] + math.sin(yaw) * reach)
            return [('corner', i, p) for i, p in enumerate(corners)] + [('rotate', 0, rot)]
        if d['type'] == 'circle':
            return [('corner', 0, (d['x'] + d['radius'], d['y']))]
        return [('point', i, tuple(p)) for i, p in enumerate(d['points'])]

    def boundingRect(self):
        margin = (HANDLE_PX * 2 + 30) / self.editor.view.scale_px()
        rect = self.outline().boundingRect()
        for _, _, (x, y) in self.handles():
            rect = rect.united(QRectF(scene_point(x, y), scene_point(x, y)))
        return rect.adjusted(-margin, -margin, margin, margin)

    def shape(self):
        path = self.outline()
        if self.isSelected():
            r = HANDLE_PX * 1.5 / self.editor.view.scale_px()
            for _, _, (x, y) in self.handles():
                path.addEllipse(scene_point(x, y), r, r)
        return path

    def paint(self, painter, _option, _widget=None):
        d = self.shape_data
        fill, edge = COLORS[self.editor.category(d)]
        chosen = self.isSelected()
        pen = QPen(edge.darker(130) if chosen else edge, 2.5 if chosen else 1.5)
        pen.setCosmetic(True)
        if self.editor.category(d) == 'none':
            pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(fill)
        painter.drawPath(self.outline())
        if self.editor.category(d) == 'nav':             # LiDAR に映らない物は斜線
            hatch = QBrush(edge, Qt.BDiagPattern)
            hatch.setTransform(QTransform().scale(1 / self.editor.view.scale_px(),
                                                  1 / self.editor.view.scale_px()))
            painter.setBrush(hatch)
            painter.drawPath(self.outline())
        if self.isSelected():
            r = HANDLE_PX / self.editor.view.scale_px()
            for kind, _, (x, y) in self.handles():
                painter.setPen(QPen(INK, 0))
                painter.setBrush(COLORS['lidar'][1] if kind == 'rotate' else Qt.white)
                p = scene_point(x, y)
                if kind == 'rotate':
                    painter.drawEllipse(p, r, r)
                else:
                    painter.drawRect(QRectF(p.x() - r, p.y() - r, 2 * r, 2 * r))

    def handle_at(self, scene_pos):
        r = HANDLE_PX * 1.6 / self.editor.view.scale_px()
        for kind, i, (x, y) in self.handles():
            p = scene_point(x, y)
            if abs(p.x() - scene_pos.x()) <= r and abs(p.y() - scene_pos.y()) <= r:
                return kind, i
        return None

    def mousePressEvent(self, event):
        if self.isSelected() and event.button() == Qt.LeftButton:
            self.drag = self.handle_at(event.scenePos())
            if self.drag:
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if not self.drag:
            super().mouseMoveEvent(event)
            return
        x, y = field_point(event.scenePos())
        x, y = self.editor.snap(x), self.editor.snap(y)
        d, (kind, i) = self.shape_data, self.drag
        self.prepareGeometryChange()
        if d['type'] == 'box' and kind == 'corner':     # まん中を動かさずに、大きさを変える
            yaw = math.radians(d.get('yaw_deg', 0.0))
            dx, dy = x - d['x'], y - d['y']
            lx = math.cos(yaw) * dx + math.sin(yaw) * dy
            ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
            d['size_x'], d['size_y'] = max(0.005, 2 * abs(lx)), max(0.005, 2 * abs(ly))
        elif kind == 'rotate':
            yaw = math.degrees(math.atan2(y - d['y'], x - d['x']))
            if self.editor.snap_step > 0:
                yaw = round(yaw / 5.0) * 5.0                # 吸着しているときは 5° ずつ
            d['yaw_deg'] = round(yaw, 2)
        elif d['type'] == 'circle':
            d['radius'] = max(0.005, math.hypot(x - d['x'], y - d['y']))
        else:
            d['points'][i] = [x, y]
        self.update_label()
        self.update()
        self.editor.show_selection()

    def mouseReleaseEvent(self, event):
        if self.drag:
            self.drag = None
            self.editor.commit()
            return
        super().mouseReleaseEvent(event)
        # いっしょに選んでいた図形も Qt が動かしているので、全部の動いた分を図形の座標に足す
        self.editor.apply_moves()

    def apply_move(self):
        """ドラッグで動いた分(pos)を図形の座標に足して、pos を 0 に戻す。動いていれば True."""
        moved = self.pos()
        if not (moved.x() or moved.y()):
            return False
        dx, dy = self.editor.snap_delta(moved.x(), -moved.y(), self.center())
        self.prepareGeometryChange()
        self.setPos(0, 0)
        d = self.shape_data
        if d['type'] == 'polygon':
            d['points'] = [[px + dx, py + dy] for px, py in d['points']]
        else:
            d['x'], d['y'] = d['x'] + dx, d['y'] + dy
        self.update_label()
        return True

    def hoverMoveEvent(self, event):
        kind = self.handle_at(event.scenePos()) if self.isSelected() else None
        self.setCursor(Qt.CrossCursor if kind else Qt.SizeAllCursor)


# ---------------------------------------------------------------- 画面(ビュー)

class FieldView(QGraphicsView):
    """フィールドを上から見た図。ホイールで拡大・縮小、右ドラッグ(または中ボタン)で動かす."""

    def __init__(self, editor, scene):
        super().__init__(scene)
        self.editor = editor
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.setBackgroundBrush(CANVAS)
        self.setMouseTracking(True)
        self.pan_from = None
        self.scale(150, 150)                    # 1 m を 150 点で表示

    def scale_px(self):
        return max(1e-6, abs(self.transform().m11()))

    def wheelEvent(self, event):
        factor = 1.2 if event.angleDelta().y() > 0 else 1 / 1.2
        if 5 < self.scale_px() * factor < 20000:
            self.scale(factor, factor)
            self.editor.refresh_items()

    def drawBackground(self, painter, rect):
        super().drawBackground(painter, rect)
        step = 0.5 if self.scale_px() > 60 else 1.0
        pen = QPen(GRID, 0)
        painter.setPen(pen)
        x = math.floor(rect.left() / step) * step
        while x < rect.right():
            painter.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
            x += step
        y = math.floor(rect.top() / step) * step
        while y < rect.bottom():
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            y += step
        axis = QPen(AXIS, 0)
        painter.setPen(axis)
        painter.drawLine(QPointF(rect.left(), 0), QPointF(rect.right(), 0))
        painter.drawLine(QPointF(0, rect.top()), QPointF(0, rect.bottom()))

    def drawForeground(self, painter, rect):
        info = self.editor.info
        (x0, x1), (y0, y1) = info['x_range'], info['y_range']
        pen = QPen(INK, 1.5, Qt.DashLine)
        pen.setCosmetic(True)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(QRectF(scene_point(x0, y1), scene_point(x1, y0)))
        self.editor.draw_tool_preview(painter)

    def mousePressEvent(self, event):
        if event.button() in (Qt.RightButton, Qt.MiddleButton):
            self.pan_from = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            return
        scene_pos = self.mapToScene(event.pos())
        if event.button() == Qt.LeftButton and self.editor.tool_press(scene_pos):
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        scene_pos = self.mapToScene(event.pos())
        x, y = field_point(scene_pos)
        self.editor.statusBar().showMessage(
            f'x {x:+.3f} m    y {y:+.3f} m    {self.editor.hint()}')
        if self.pan_from is not None:
            delta = event.pos() - self.pan_from
            self.pan_from = event.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            return
        if self.editor.tool_move(scene_pos):
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self.pan_from is not None:
            self.pan_from = None
            self.unsetCursor()
            return
        scene_pos = self.mapToScene(event.pos())
        if event.button() == Qt.LeftButton and self.editor.tool_release(scene_pos):
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event):
        if not self.editor.tool_double_click():
            super().mouseDoubleClickEvent(event)


# ---------------------------------------------------------------- ウィンドウ

class FieldEditor(QMainWindow):

    def __init__(self, path=None):
        super().__init__()
        self.path = None
        self.info, self.shapes = fio.new_document()
        self.undo_stack, self.undo_index = [], -1
        self.saved_state = None
        self.tool = 'select'
        self.draft = []             # 描いている途中の点(フィールドの座標)
        self.snap_step = 0.0
        self.image, self.image_path, self.calibration = None, None, None
        self.calib_clicks = []
        self.pick_color = QColor(0, 0, 0)
        self.updating_panel = False

        self.scene = QGraphicsScene(self)
        self.scene.setSceneRect(-50, -50, 100, 100)
        self.scene.selectionChanged.connect(self.show_selection)
        self.view = FieldView(self, self.scene)
        self.bg_item = QGraphicsPixmapItem()
        self.bg_item.setZValue(-10)
        self.bg_item.setTransformationMode(Qt.SmoothTransformation)
        self.scene.addItem(self.bg_item)
        self.items = []

        self.make_toolbar()
        self.make_panel()
        self.resize(1400, 860)
        if path and os.path.exists(path):
            self.open_path(path)
        else:
            if path:
                self.path = path
                self.info['name'] = os.path.splitext(os.path.basename(path))[0]
            self.reset_history()
            self.rebuild()
            self.fit_view()

    # ---- 部品 ----

    def make_toolbar(self):
        bar = QToolBar('道具')
        bar.setMovable(False)
        self.addToolBar(bar)

        def action(text, slot, shortcut=None, checkable=False):
            act = QAction(text, self)
            act.triggered.connect(slot)
            if shortcut:
                act.setShortcut(QKeySequence(shortcut))
            act.setCheckable(checkable)
            bar.addAction(act)
            return act

        action('新規', self.new_file, 'Ctrl+N')
        action('開く', self.open_dialog, 'Ctrl+O')
        action('保存', self.save, 'Ctrl+S')
        action('名前を付けて保存', self.save_as, 'Ctrl+Shift+S')
        bar.addSeparator()
        group = QActionGroup(self)
        self.tool_actions = {}
        for key, text, shortcut in (('select', '選択(V)', 'V'), ('box', '箱(B)', 'B'),
                                    ('circle', '円(C)', 'C'), ('polygon', '多角形(P)', 'P')):
            act = action(text, lambda _=False, k=key: self.set_tool(k), shortcut, True)
            group.addAction(act)
            self.tool_actions[key] = act
        self.tool_actions['select'].setChecked(True)
        bar.addSeparator()
        action('元に戻す', self.undo, 'Ctrl+Z')
        action('やり直し', self.redo, 'Ctrl+Y')
        action('複製', self.duplicate, 'Ctrl+D')
        action('消す', self.delete_selected, 'Delete')
        bar.addSeparator()
        self.snap_box = QComboBox()
        for text, value in SNAP_STEPS:
            self.snap_box.addItem(text, value)
        self.snap_box.setCurrentIndex(1)
        self.snap_box.currentIndexChanged.connect(self.set_snap)
        self.set_snap()
        bar.addWidget(QLabel(' グリッド '))
        bar.addWidget(self.snap_box)
        action('全体を表示', self.fit_view, 'F')

    def make_panel(self):
        tabs = QTabWidget()
        tabs.setMinimumWidth(380)
        tabs.addTab(self.make_shape_tab(), '図形')
        tabs.addTab(self.make_field_tab(), 'フィールド')
        tabs.addTab(self.make_image_tab(), '画像')
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(tabs)
        legend = QLabel('<b>色</b>  <span style="color:#0b84d8">■</span> 通れない・LiDAR に映る  '
                        '<span style="color:#d97706">■</span> 通れないが LiDAR に映らない<br>'
                        '<span style="color:#6d5bd8">■</span> LiDAR に映るが下をくぐれる  '
                        '<span style="color:#9aa3b2">□</span> 通れる<br>'
                        '<b>操作</b>  ホイール: 拡大・縮小 / 右ドラッグ: 動かす / F: 全体<br>'
                        '多角形: クリックで角を足し、ダブルクリック(または Enter)で終わる')
        legend.setWordWrap(True)
        layout.addWidget(legend)
        dock_host = QWidget()
        h = QHBoxLayout(dock_host)
        h.setContentsMargins(0, 0, 0, 0)
        h.addWidget(self.view, 1)
        h.addWidget(container)
        self.setCentralWidget(dock_host)

    def spin(self, low, high, decimals=3, step=0.01, suffix=' m'):
        box = QDoubleSpinBox()
        box.setRange(low, high)
        box.setDecimals(decimals)
        box.setSingleStep(step)
        box.setSuffix(suffix)
        box.setKeyboardTracking(False)
        return box

    def make_shape_tab(self):
        tab = QWidget()
        form = QFormLayout(tab)
        self.sel_label = QLabel('図形を選んでください')
        form.addRow(self.sel_label)
        self.f_name = QLineEdit()
        self.f_name.editingFinished.connect(self.panel_to_shape)
        form.addRow('名前', self.f_name)
        self.f = {}
        for key, label, low, high in (('x', '中心 x(前)', -100, 100), ('y', '中心 y(左)', -100, 100),
                                      ('size_x', '大きさ x', 0.001, 100),
                                      ('size_y', '大きさ y', 0.001, 100),
                                      ('radius', '半径', 0.001, 50)):
            self.f[key] = self.spin(low, high)
            self.f[key].valueChanged.connect(self.panel_to_shape)
            form.addRow(label, self.f[key])
        self.f['yaw_deg'] = self.spin(-360, 360, 1, 5, ' °')
        self.f['yaw_deg'].valueChanged.connect(self.panel_to_shape)
        form.addRow('回転(反時計回り)', self.f['yaw_deg'])
        self.height_mode = QComboBox()
        self.height_mode.addItems(['床から天井まで(壁・柱)', '床からの高さ', '浮いている(棚など)'])
        self.height_mode.currentIndexChanged.connect(self.panel_to_shape)
        form.addRow('高さ', self.height_mode)
        self.f['z_min'] = self.spin(-10, 100)
        self.f['z_max'] = self.spin(0, 100)
        for key in ('z_min', 'z_max'):
            self.f[key].valueChanged.connect(self.panel_to_shape)
        form.addRow('下の高さ', self.f['z_min'])
        form.addRow('上の高さ', self.f['z_max'])
        self.f_passable = QCheckBox('通れる(地図に入れない)')
        self.f_passable.toggled.connect(self.panel_to_shape)
        form.addRow(self.f_passable)
        self.shape_rows = form
        return tab

    def make_field_tab(self):
        tab = QWidget()
        form = QFormLayout(tab)
        self.fd_name = QLineEdit()
        self.fd_name.editingFinished.connect(self.panel_to_field)
        form.addRow('名前', self.fd_name)
        self.fd = {}
        for key, label, low, high, dec, step in (
                ('resolution', '1 マスの大きさ', 0.005, 1.0, 3, 0.01),
                ('x0', 'x の範囲(小)', -100, 100, 3, 0.1), ('x1', 'x の範囲(大)', -100, 100, 3, 0.1),
                ('y0', 'y の範囲(小)', -100, 100, 3, 0.1), ('y1', 'y の範囲(大)', -100, 100, 3, 0.1),
                ('border', 'まわりの壁の厚さ', 0, 1, 3, 0.01),
                ('step_height', '乗り越えられる高さ', 0, 1, 3, 0.005)):
            self.fd[key] = self.spin(low, high, dec, step)
            self.fd[key].valueChanged.connect(self.panel_to_field)
            form.addRow(label, self.fd[key])
        self.fd_lidar = QLineEdit()
        self.fd_lidar.setPlaceholderText('例: 0.2(複数なら 0.2, 0.5)。空なら全部映る')
        self.fd_lidar.editingFinished.connect(self.panel_to_field)
        form.addRow('LiDAR の高さ [m]', self.fd_lidar)
        self.fd_robot = QLineEdit()
        self.fd_robot.setPlaceholderText('例: 0.6。空なら上の物もくぐれない')
        self.fd_robot.editingFinished.connect(self.panel_to_field)
        form.addRow('ロボットの高さ [m]', self.fd_robot)
        fit = QPushButton('図形が全部入る広さにする')
        fit.clicked.connect(self.fit_field_to_shapes)
        form.addRow(fit)
        return tab

    def make_image_tab(self):
        tab = QWidget()
        form = QFormLayout(tab)
        load = QPushButton('画像を開く(フィールドの図・地図)')
        load.clicked.connect(self.load_image_dialog)
        form.addRow(load)
        self.img_info = QLabel('画像はまだありません')
        self.img_info.setWordWrap(True)
        form.addRow(self.img_info)
        self.img_show = QCheckBox('表示する')
        self.img_show.setChecked(True)
        self.img_show.toggled.connect(lambda on: self.bg_item.setVisible(on))
        form.addRow(self.img_show)
        self.img_opacity = QSlider(Qt.Horizontal)
        self.img_opacity.setRange(5, 100)
        self.img_opacity.setValue(60)
        self.img_opacity.valueChanged.connect(lambda v: self.bg_item.setOpacity(v / 100))
        form.addRow('濃さ', self.img_opacity)
        calib = QPushButton('2 つの点で位置を合わせる')
        calib.clicked.connect(lambda: self.set_tool('calibrate'))
        form.addRow(calib)
        form.addRow(QLabel('<b>画像から図形を作る</b>'))
        self.img_mode = QComboBox()
        self.img_mode.addItems(['暗いところ', '選んだ色に近いところ'])
        form.addRow('取り出す所', self.img_mode)
        self.img_threshold = QSpinBox()
        self.img_threshold.setRange(1, 254)
        self.img_threshold.setValue(100)
        form.addRow('暗さのしきい値(0〜255)', self.img_threshold)
        color_row = QHBoxLayout()
        self.color_btn = QPushButton('色を選ぶ')
        self.color_btn.clicked.connect(self.choose_color)
        pick = QPushButton('画像から拾う')
        pick.clicked.connect(lambda: self.set_tool('pick'))
        color_row.addWidget(self.color_btn)
        color_row.addWidget(pick)
        form.addRow('色', color_row)
        self.img_tolerance = QSpinBox()
        self.img_tolerance.setRange(1, 255)
        self.img_tolerance.setValue(60)
        form.addRow('色の近さ', self.img_tolerance)
        self.img_min = QSpinBox()
        self.img_min.setRange(1, 10000)
        self.img_min.setValue(6)
        self.img_min.setSuffix(' マス')
        form.addRow('これより小さい物は捨てる', self.img_min)
        make = QPushButton('図形を作る(フィールドの範囲の中だけ)')
        make.clicked.connect(self.image_to_shapes)
        form.addRow(make)
        self.update_color_button()
        return tab

    # ---- 状態 ----

    def category(self, shape):
        """図形の色分け: both / nav / lidar / none."""
        if shape.get('passable'):
            return 'none'
        s = to_field_shape(shape)
        field = self.field_settings()
        if s is None or field is None:
            return 'both'
        nav, lidar = field.blocks(s, fm.NAV), field.blocks(s, fm.LIDAR)
        return {(True, True): 'both', (True, False): 'nav', (False, True): 'lidar'}.get(
            (nav, lidar), 'none')

    def field_settings(self):
        try:
            return fm.parse_field({'field': {k: v for k, v in self.info.items()
                                             if k != 'background'}, 'shapes': []})
        except fm.FieldError:
            return None

    def state(self):
        return json.dumps([self.info, self.shapes], ensure_ascii=False, sort_keys=True)

    def reset_history(self):
        self.undo_stack, self.undo_index = [self.state()], 0
        self.saved_state = self.state()
        self.update_title()

    def commit(self):
        """変更を 1 つ記録する(元に戻せるように)."""
        if not self.undo_stack:         # 開いている途中(あとで reset_history する)
            return
        current = self.state()
        if current != self.undo_stack[self.undo_index]:
            self.undo_stack = self.undo_stack[:self.undo_index + 1] + [current]
            self.undo_stack = self.undo_stack[-200:]
            self.undo_index = len(self.undo_stack) - 1
        self.update_title()
        self.refresh_items()
        self.show_selection()

    def restore(self, state):
        selected = self.selected_indexes()
        self.info, self.shapes = json.loads(state)
        self.rebuild([i for i in selected if i < len(self.shapes)])
        self.update_title()

    def undo(self):
        if self.undo_index > 0:
            self.undo_index -= 1
            self.restore(self.undo_stack[self.undo_index])

    def redo(self):
        if self.undo_index < len(self.undo_stack) - 1:
            self.undo_index += 1
            self.restore(self.undo_stack[self.undo_index])

    def dirty(self):
        return self.state() != self.saved_state

    def update_title(self):
        name = self.path or '(まだ保存していない)'
        self.setWindowTitle(f'{"*" if self.dirty() else ""}フィールドエディタ - {name}')

    # ---- 図形の絵 ----

    def rebuild(self, select=()):
        """図形の絵を全部作り直す."""
        for item in self.items:
            self.scene.removeItem(item)
        self.items = [ShapeItem(self, i) for i in range(len(self.shapes))]
        for item in self.items:
            self.scene.addItem(item)
        for i in select:
            self.items[i].setSelected(True)
        self.load_field_panel()
        self.show_selection()

    def refresh_items(self):
        for item in self.items:
            item.prepareGeometryChange()
            item.update_label()
            item.update()
        self.scene.update()

    def apply_moves(self):
        """ドラッグで動いた図形を全部、図形の座標に反映して、変更を 1 つ記録する."""
        moved = [item.apply_move() for item in self.items]     # 全部に反映してから判定する
        if any(moved):
            self.commit()

    def selected_indexes(self):
        return sorted(item.index for item in self.items if item.isSelected())

    def fit_view(self):
        (x0, x1), (y0, y1) = self.info['x_range'], self.info['y_range']
        rect = QRectF(scene_point(x0, y1), scene_point(x1, y0))
        self.view.fitInView(rect.adjusted(-0.3, -0.3, 0.3, 0.3), Qt.KeepAspectRatio)
        self.refresh_items()

    # ---- 吸着 ----

    def set_snap(self):
        self.snap_step = self.snap_box.currentData()

    def snap(self, value):
        if self.snap_step <= 0:
            return value
        return round(round(value / self.snap_step) * self.snap_step, 6)

    def snap_delta(self, dx, dy, center):
        """動かした量を、動かしたあとの中心がグリッドに乗るように直す."""
        if self.snap_step <= 0:
            return dx, dy
        cx, cy = center
        return self.snap(cx + dx) - cx, self.snap(cy + dy) - cy

    # ---- 道具 ----

    def set_tool(self, tool):
        self.tool = tool
        self.draft = []
        self.calib_clicks = [] if tool == 'calibrate' else self.calib_clicks
        if tool in self.tool_actions:
            self.tool_actions[tool].setChecked(True)
        else:
            for act in self.tool_actions.values():
                act.setChecked(False)
        drawing = tool != 'select'
        self.view.setDragMode(QGraphicsView.NoDrag if drawing else QGraphicsView.RubberBandDrag)
        for item in self.items:
            item.setFlag(QGraphicsItem.ItemIsMovable, not drawing)
            item.setFlag(QGraphicsItem.ItemIsSelectable, not drawing)
        if tool in ('calibrate', 'pick') and self.image is None:
            QMessageBox.information(self, '画像がありません', '先に「画像」の欄で画像を開いてください')
            self.set_tool('select')
            return
        self.view.setCursor(Qt.ArrowCursor if tool == 'select' else Qt.CrossCursor)
        self.statusBar().showMessage(self.hint())
        self.view.viewport().update()

    def hint(self):
        return {'select': 'クリックで選ぶ / ドラッグで動かす / 角の点で大きさ、丸い点で回転',
                'box': 'ドラッグして箱を描く(Esc でやめる)',
                'circle': '中心からドラッグして円を描く',
                'polygon': 'クリックで角を足す。ダブルクリックか Enter で終わる(Esc でやめる)',
                'calibrate': f'画像の上で、座標が分かる点をクリック({len(self.calib_clicks) + 1} / 2 点目)',
                'pick': '画像の上で、取り出したい色の場所をクリック'}.get(self.tool, '')

    def tool_press(self, scene_pos):
        if self.tool == 'select':
            return False
        x, y = field_point(scene_pos)
        point = (self.snap(x), self.snap(y))
        if self.tool in ('box', 'circle'):
            self.draft = [point, point]
        elif self.tool == 'polygon':
            if not self.draft:
                self.draft = [point, point]
            else:
                self.draft[-1] = point
                self.draft.append(point)
        elif self.tool == 'calibrate':
            self.calibrate_click(x, y)
        elif self.tool == 'pick':
            self.pick_from_image(x, y)
        self.view.viewport().update()
        return True

    def tool_move(self, scene_pos):
        if self.tool not in ('box', 'circle', 'polygon') or not self.draft:
            return False
        x, y = field_point(scene_pos)
        self.draft[-1] = (self.snap(x), self.snap(y))
        self.view.viewport().update()
        return True

    def tool_release(self, scene_pos):
        if self.tool not in ('box', 'circle') or len(self.draft) != 2:
            return self.tool != 'select'
        (x0, y0), (x1, y1) = self.draft
        self.draft = []
        r6 = lambda v: round(v, 6)              # noqa: E731  引き算の端数(0.39999…)をなくす
        if self.tool == 'box' and abs(x1 - x0) > 0.004 and abs(y1 - y0) > 0.004:
            self.add_shapes([{'type': 'box', 'name': '', 'x': r6((x0 + x1) / 2),
                              'y': r6((y0 + y1) / 2), 'size_x': r6(abs(x1 - x0)),
                              'size_y': r6(abs(y1 - y0)), 'yaw_deg': 0.0}])
        elif self.tool == 'circle' and math.hypot(x1 - x0, y1 - y0) > 0.004:
            self.add_shapes([{'type': 'circle', 'name': '', 'x': x0, 'y': y0,
                              'radius': r6(math.hypot(x1 - x0, y1 - y0))}])
        self.view.viewport().update()
        return True

    def tool_double_click(self):
        if self.tool == 'polygon':
            self.finish_polygon()
            return True
        return False

    def finish_polygon(self):
        points = []
        for p in self.draft:                        # 同じ点が続いたら 1 つにする
            if not points or math.hypot(p[0] - points[-1][0], p[1] - points[-1][1]) > 1e-4:
                points.append(p)
        self.draft = []
        if len(points) >= 3:
            self.add_shapes([{'type': 'polygon', 'name': '',
                              'points': [[x, y] for x, y in points]}])
        self.view.viewport().update()

    def draw_tool_preview(self, painter):
        if not self.draft:
            return
        pen = QPen(ACCENT, 1.5, Qt.DashLine)
        pen.setCosmetic(True)
        painter.setPen(pen)
        painter.setBrush(QColor(11, 132, 216, 40))
        if self.tool == 'box':
            (x0, y0), (x1, y1) = self.draft
            painter.drawRect(QRectF(scene_point(x0, y0), scene_point(x1, y1)).normalized())
        elif self.tool == 'circle':
            (x0, y0), (x1, y1) = self.draft
            r = math.hypot(x1 - x0, y1 - y0)
            painter.drawEllipse(scene_point(x0, y0), r, r)
        elif self.tool == 'polygon':
            painter.drawPolyline(QPolygonF([scene_point(x, y) for x, y in self.draft]))

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Return, Qt.Key_Enter) and self.tool == 'polygon':
            self.finish_polygon()
        elif event.key() == Qt.Key_Escape:
            self.set_tool('select')
        else:
            super().keyPressEvent(event)

    # ---- 図形の追加・削除 ----

    def add_shapes(self, shapes):
        start = len(self.shapes)
        self.shapes.extend(shapes)
        self.rebuild(range(start, len(self.shapes)))
        self.commit()

    def delete_selected(self):
        selected = set(self.selected_indexes())
        if not selected:
            return
        self.shapes = [s for i, s in enumerate(self.shapes) if i not in selected]
        self.rebuild()
        self.commit()

    def duplicate(self):
        copies = []
        for i in self.selected_indexes():
            s = copy.deepcopy(self.shapes[i])
            offset = 0.1
            if s['type'] == 'polygon':
                s['points'] = [[x + offset, y - offset] for x, y in s['points']]
            else:
                s['x'], s['y'] = s['x'] + offset, s['y'] - offset
            copies.append(s)
        if copies:
            self.add_shapes(copies)

    # ---- 右の欄 ----

    def show_selection(self):
        selected = self.selected_indexes()
        self.updating_panel = True
        enabled = len(selected) == 1
        for w in list(self.f.values()) + [self.f_name, self.height_mode, self.f_passable]:
            w.setEnabled(enabled)
        if not selected:
            self.sel_label.setText('図形を選んでください(描くときは上の「箱」「円」「多角形」)')
        elif len(selected) > 1:
            self.sel_label.setText(f'{len(selected)} 個選んでいます(動かす・複製・消すができます)')
        else:
            s = self.shapes[selected[0]]
            kinds = {'box': '箱', 'circle': '円', 'polygon': f'多角形(角 {len(s.get("points", []))} 個)'}
            self.sel_label.setText(f'<b>{kinds[s["type"]]}</b>')
            self.f_name.setText(s.get('name', ''))
            for key in ('x', 'y', 'size_x', 'size_y', 'radius', 'yaw_deg'):
                show = key in s
                self.f[key].setEnabled(show)
                if show:
                    self.f[key].setValue(s[key])
            if s['type'] == 'polygon':
                cx, cy = self.items[selected[0]].center()
                self.f['x'].setValue(cx)
                self.f['y'].setValue(cy)
                self.f['x'].setEnabled(True)
                self.f['y'].setEnabled(True)
            z_min, z_max = s.get('z_min', 0.0), s.get('z_max')
            mode = 0 if z_max is None else (1 if z_min == 0.0 else 2)
            self.height_mode.setCurrentIndex(mode)
            self.f['z_min'].setValue(z_min)
            self.f['z_max'].setValue(z_max if z_max is not None else 0.3)
            self.f['z_min'].setEnabled(mode == 2)
            self.f['z_max'].setEnabled(mode > 0)
            self.f_passable.setChecked(bool(s.get('passable')))
        self.updating_panel = False

    def panel_to_shape(self):
        if self.updating_panel:
            return
        selected = self.selected_indexes()
        if len(selected) != 1:
            return
        i = selected[0]
        s = self.shapes[i]
        s['name'] = self.f_name.text().strip()
        if s['type'] == 'polygon':                  # 多角形は中心を動かした分だけ平行移動
            cx, cy = self.items[i].center()
            dx, dy = self.f['x'].value() - cx, self.f['y'].value() - cy
            s['points'] = [[x + dx, y + dy] for x, y in s['points']]
        for key in ('x', 'y', 'size_x', 'size_y', 'radius', 'yaw_deg'):
            if key in s:
                s[key] = self.f[key].value()
        mode = self.height_mode.currentIndex()
        s.pop('z_min', None)
        s.pop('z_max', None)
        if mode >= 1:
            s['z_max'] = self.f['z_max'].value()
        if mode == 2:
            s['z_min'] = min(self.f['z_min'].value(), s['z_max'] - 0.001)
        if self.f_passable.isChecked():
            s['passable'] = True
        else:
            s.pop('passable', None)
        self.commit()

    def load_field_panel(self):
        self.updating_panel = True
        info = self.info
        self.fd_name.setText(str(info.get('name', '')))
        self.fd['resolution'].setValue(info.get('resolution', 0.05))
        self.fd['x0'].setValue(info['x_range'][0])
        self.fd['x1'].setValue(info['x_range'][1])
        self.fd['y0'].setValue(info['y_range'][0])
        self.fd['y1'].setValue(info['y_range'][1])
        self.fd['border'].setValue(info.get('border', info.get('resolution', 0.05)))
        self.fd['step_height'].setValue(info.get('step_height', 0.0))
        lidar = info.get('lidar_height')
        lidar = lidar if isinstance(lidar, list) else ([] if lidar is None else [lidar])
        self.fd_lidar.setText(', '.join(f'{v:g}' for v in lidar))
        robot = info.get('robot_height')
        self.fd_robot.setText('' if robot is None else f'{robot:g}')
        self.updating_panel = False

    def panel_to_field(self):
        if self.updating_panel:
            return
        info = self.info
        info['name'] = self.fd_name.text().strip() or 'フィールド'
        info['resolution'] = self.fd['resolution'].value()
        x0, x1 = sorted((self.fd['x0'].value(), self.fd['x1'].value()))
        y0, y1 = sorted((self.fd['y0'].value(), self.fd['y1'].value()))
        if x1 - x0 < 0.05 or y1 - y0 < 0.05:
            self.load_field_panel()
            return
        info['x_range'], info['y_range'] = [x0, x1], [y0, y1]
        info['border'] = self.fd['border'].value()
        step = self.fd['step_height'].value()
        if step > 0:
            info['step_height'] = step
        else:
            info.pop('step_height', None)
        try:
            values = [float(v) for v in self.fd_lidar.text().replace('、', ',').split(',')
                      if v.strip()]
            robot = self.fd_robot.text().strip()
            robot = float(robot) if robot else None
        except ValueError:
            QMessageBox.warning(self, '数を入れてください', 'LiDAR とロボットの高さは、数(m)で入れてください')
            self.load_field_panel()
            return
        if values:
            info['lidar_height'] = values[0] if len(values) == 1 else values
        else:
            info.pop('lidar_height', None)
        if robot:
            info['robot_height'] = robot
        else:
            info.pop('robot_height', None)
        if self.field_settings() is None:
            QMessageBox.warning(self, '設定がおかしい', 'ロボットの高さは、乗り越えられる高さより大きくしてください')
        self.commit()
        self.view.viewport().update()

    def fit_field_to_shapes(self):
        if not self.shapes:
            return
        shapes = [to_field_shape(s) for s in self.shapes]
        boxes = [s.bounds() for s in shapes if s is not None]
        margin = 0.2
        self.info['x_range'] = [round(min(b[0] for b in boxes) - margin, 3),
                                round(max(b[1] for b in boxes) + margin, 3)]
        self.info['y_range'] = [round(min(b[2] for b in boxes) - margin, 3),
                                round(max(b[3] for b in boxes) + margin, 3)]
        self.load_field_panel()
        self.commit()
        self.fit_view()

    # ---- 画像 ----

    def load_image_dialog(self):
        path, _ = QFileDialog.getOpenFileName(
            self, '画像を開く', os.path.dirname(self.path or fm.SOURCE_FIELDS),
            '画像・地図 (*.png *.jpg *.jpeg *.bmp *.pgm *.yaml);;すべて (*)')
        if path:
            self.load_image(path)

    def load_image(self, path, points=None, opacity=None):
        """画像を下に敷く。map_server の地図(.yaml)なら、縮尺と位置もそのまま合わせる."""
        calibration = None
        if path.endswith(('.yaml', '.yml')):
            try:
                with open(path, encoding='utf-8') as f:
                    meta = yaml.safe_load(f)
                image_path = os.path.join(os.path.dirname(path), meta['image'])
                image = QImage(image_path)
                if not image.isNull():
                    calibration = fio.Calibration.from_map_yaml(meta, image.height())
                path = image_path
            except (OSError, KeyError, TypeError, ValueError) as e:
                QMessageBox.warning(self, '読めません', f'{path}: {e}')
                return False
        else:
            image = QImage(path)
        if image.isNull():
            QMessageBox.warning(self, '読めません', f'{path} は画像として読めませんでした')
            return False
        self.image = image.convertToFormat(QImage.Format_ARGB32)
        self.image_path = os.path.abspath(path)
        self.bg_item.setPixmap(QPixmap.fromImage(self.image))
        if points:
            try:
                calibration = fio.Calibration.from_points(points)
            except (ValueError, TypeError):
                calibration = None
        self.calibration = calibration or fio.Calibration.fit(
            image.width(), image.height(), self.info['x_range'], self.info['y_range'])
        if opacity is not None:
            self.img_opacity.setValue(int(opacity * 100))
        self.apply_calibration()
        return True

    def apply_calibration(self):
        a, b = self.calibration.a, self.calibration.b
        # 画素 (u, v) → シーン (x, -y)。x = ar u + ai v + br、-y = -ai u + ar v - bi
        self.bg_item.setTransform(QTransform(a.real, -a.imag, a.imag, a.real, b.real, -b.imag))
        self.bg_item.setOpacity(self.img_opacity.value() / 100)
        cal = self.calibration
        self.img_info.setText(f'{os.path.basename(self.image_path)}({self.image.width()} × '
                              f'{self.image.height()} 画素)<br>'
                              f'1 画素 = {cal.meters_per_pixel * 1000:.2f} mm、'
                              f'回転 {cal.rotation_deg:+.1f}°')
        self.info['background'] = {'image': self.image_ref(),
                                   'opacity': round(self.img_opacity.value() / 100, 2),
                                   'points': cal.points(self.image.width())}
        self.commit()

    def image_ref(self):
        """保存するときの画像の場所(YAML からの相対パス)."""
        if self.path and self.image_path:
            return os.path.relpath(self.image_path, os.path.dirname(os.path.abspath(self.path)))
        return self.image_path

    def calibrate_click(self, x, y):
        item_pos = self.bg_item.mapFromScene(scene_point(x, y))
        u, v = item_pos.x(), item_pos.y()
        if not (0 <= u <= self.image.width() and 0 <= v <= self.image.height()):
            self.statusBar().showMessage('画像の上をクリックしてください')
            return
        text, ok = QInputDialog.getText(
            self, f'位置合わせ({len(self.calib_clicks) + 1} / 2 点目)',
            'クリックした場所の、フィールドの座標 x, y [m] を入れてください(例: 3.0, -2.0)',
            text=f'{x:.3f}, {y:.3f}')
        if not ok:
            self.set_tool('select')
            return
        try:
            fx, fy = (float(v) for v in text.replace('、', ',').split(','))
        except ValueError:
            QMessageBox.warning(self, '数を入れてください', '「x, y」の形で入れてください')
            return
        self.calib_clicks.append([u, v, fx, fy])
        if len(self.calib_clicks) == 2:
            try:
                self.calibration = fio.Calibration.from_points(self.calib_clicks)
            except ValueError as e:
                QMessageBox.warning(self, '合わせられません', str(e))
            else:
                self.apply_calibration()
            self.set_tool('select')
        else:
            self.statusBar().showMessage(self.hint())

    def pixel_at(self, x, y):
        """フィールドの座標の画素の色(画像の外なら None)."""
        u, v = self.calibration.to_pixel(x, y)
        if 0 <= u < self.image.width() and 0 <= v < self.image.height():
            return QColor.fromRgba(self.image.pixel(int(u), int(v)))
        return None

    def pick_from_image(self, x, y):
        color = self.pixel_at(x, y)
        if color is not None:
            self.pick_color = color
            self.img_mode.setCurrentIndex(1)
            self.update_color_button()
        self.set_tool('select')

    def choose_color(self):
        color = QColorDialog.getColor(self.pick_color, self, '取り出す色')
        if color.isValid():
            self.pick_color = color
            self.img_mode.setCurrentIndex(1)
            self.update_color_button()

    def update_color_button(self):
        c = self.pick_color
        text = '#ffffff' if c.lightness() < 128 else '#000000'
        self.color_btn.setStyleSheet(f'background: {c.name()}; color: {text};')

    def image_mask(self):
        """フィールドの範囲をマス目に分けて、取り出す所かどうか(row 0 が下)."""
        (x0, x1), (y0, y1) = self.info['x_range'], self.info['y_range']
        step = max(0.01, min(0.2, self.info.get('resolution', 0.05)))
        cols, rows = int(math.ceil((x1 - x0) / step)), int(math.ceil((y1 - y0) / step))
        dark = self.img_mode.currentIndex() == 0
        threshold, tolerance = self.img_threshold.value(), self.img_tolerance.value()
        target = self.pick_color
        grid = []
        for r in range(rows):
            row = []
            for c in range(cols):
                color = self.pixel_at(x0 + (c + 0.5) * step, y0 + (r + 0.5) * step)
                if color is None or color.alpha() < 128:
                    row.append(False)
                elif dark:
                    row.append(0.299 * color.red() + 0.587 * color.green()
                               + 0.114 * color.blue() < threshold)
                else:
                    row.append(math.sqrt((color.red() - target.red()) ** 2
                                         + (color.green() - target.green()) ** 2
                                         + (color.blue() - target.blue()) ** 2) < tolerance)
            grid.append(row)
        return grid, x0, y0, step

    def image_to_shapes(self):
        if self.image is None:
            QMessageBox.information(self, '画像がありません', '先に画像を開いてください')
            return
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            grid, x0, y0, step = self.image_mask()
            shapes = fio.grid_to_shapes(grid, x0, y0, step, min_cells=self.img_min.value())
        finally:
            QApplication.restoreOverrideCursor()
        if not shapes:
            QMessageBox.information(self, '見つかりません',
                                    'あてはまる所がありませんでした。しきい値や色を変えてみてください')
            return
        if len(shapes) > 300:
            answer = QMessageBox.question(self, '多すぎます', f'{len(shapes)} 個の図形ができます。足しますか'
                                          '(小さい物を捨てる大きさを増やすと減ります)')
            if answer != QMessageBox.Yes:
                return
        self.add_shapes(shapes)
        self.statusBar().showMessage(f'{len(shapes)} 個の図形を足しました(元に戻すは Ctrl+Z)')

    # ---- ファイル ----

    def ask_save(self):
        """変更があれば、保存するか聞く。続けてよければ True."""
        if not self.dirty():
            return True
        answer = QMessageBox.question(self, '保存していない変更があります', '保存しますか',
                                      QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel)
        if answer == QMessageBox.Save:
            return self.save()
        return answer == QMessageBox.Discard

    def new_file(self):
        if not self.ask_save():
            return
        self.path = None
        self.info, self.shapes = fio.new_document()
        self.clear_image()
        self.rebuild()
        self.reset_history()
        self.fit_view()

    def clear_image(self):
        self.image = self.image_path = self.calibration = None
        self.bg_item.setPixmap(QPixmap())
        self.img_info.setText('画像はまだありません')

    def open_dialog(self):
        if not self.ask_save():
            return
        start = os.path.dirname(self.path) if self.path else fm.SOURCE_FIELDS
        path, _ = QFileDialog.getOpenFileName(self, 'フィールドを開く', start,
                                              'フィールド (*.yaml *.yml *.json);;すべて (*)')
        if path:
            self.open_path(path)

    def open_path(self, path):
        try:
            info, shapes = fio.load_document(path)
        except fm.FieldError as e:
            QMessageBox.warning(self, '開けません', str(e))
            return False
        self.path = path if not path.endswith('.json') else None    # JSON は YAML で保存し直す
        self.info, self.shapes = info, shapes
        self.clear_image()
        background = info.get('background')
        self.rebuild()
        if isinstance(background, dict) and background.get('image'):
            image = os.path.join(os.path.dirname(os.path.abspath(path)), background['image'])
            if not self.load_image(image, background.get('points'), background.get('opacity')):
                self.info['background'] = background    # 画像がなくても設定は消さない
        self.reset_history()
        self.fit_view()
        return True

    def save(self):
        if not self.path:
            return self.save_as()
        if self.field_settings() is None:
            QMessageBox.warning(self, '保存できません', 'フィールドの設定がおかしいところがあります')
            return False
        if self.image_path:
            self.info['background']['image'] = self.image_ref()
            self.info['background']['opacity'] = round(self.img_opacity.value() / 100, 2)
        try:
            text = fio.dump_document(self.info, self.shapes)
            with open(self.path, 'w', encoding='utf-8') as f:
                f.write(text)
        except OSError as e:
            QMessageBox.critical(self, '保存できませんでした', str(e))
            return False
        self.saved_state = self.state()
        self.update_title()
        self.statusBar().showMessage(f'保存しました: {self.path}(地図のノードが動いていれば、すぐ反映されます)')
        return True

    def save_as(self):
        start = self.path or os.path.join(fm.SOURCE_FIELDS, 'new_field.yaml')
        path, _ = QFileDialog.getSaveFileName(self, '名前を付けて保存', start, 'フィールド (*.yaml)')
        if not path:
            return False
        if not path.endswith(('.yaml', '.yml')):
            path += '.yaml'
        self.path = path
        return self.save()

    def closeEvent(self, event):
        if self.ask_save():
            event.accept()
        else:
            event.ignore()


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description='フィールドエディタ')
    parser.add_argument('field', nargs='?', default='example',
                        help='fields/ の中の名前か、ファイルのパス(なければ新しく作る)')
    args, _ = parser.parse_known_args(argv)
    path = fm.resolve_field_path(args.field)
    if not os.path.exists(path) and os.sep not in args.field:
        path = os.path.join(fm.SOURCE_FIELDS if os.path.isdir(fm.SOURCE_FIELDS)
                            else os.getcwd(), args.field + '.yaml')
    app = QApplication(sys.argv[:1])
    editor = FieldEditor(path)
    editor.show()
    app.exec_()
