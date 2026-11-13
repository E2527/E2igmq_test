"""
ロボットコンソールの色・フォントと、絵を描くための小さな部品(PyQt5).

見た目を変えたいときは、まずここの色を変える。
"""

import math

from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QColor, QFont, QPen, QPolygonF

from . import console_state as cs

# ---- 色(暗い背景に、明るい文字と差し色) ----
BG = '#0a0e15'          # ウィンドウの背景
CARD = '#121823'        # カードの背景
EDGE = '#1f2836'        # カードのふち、目盛りの線
TRACK = '#1a2230'       # バーの溝、押していないキー
TEXT = '#e9eef6'        # ふつうの文字
DIM = '#8c97ab'         # 補足の文字
FAINT = '#465166'       # さらに薄い文字、細い線
ACCENT = '#38c6ff'      # 差し色(水色): 実際の速さ、軌跡
ACCENT_2 = '#8b7bff'    # 差し色(紫): 回転、グラデーションの端
GOOD = '#34d399'        # OK、実際の矢印
WARN = '#fbbf24'        # 注意、頭打ち
BAD = '#fb5c72'         # エラー、停止ボタン
STALE = '#5d6779'       # 届いていない

LEVEL_COLOR = {cs.OK: GOOD, cs.WARN: WARN, cs.ERROR: BAD, cs.STALE: STALE}

FONT_FAMILY = 'Noto Sans CJK JP'    # なければ、OS が近いフォントを選ぶ


def font(size, bold=False, spacing=0.0, scale=1.0):
    """
    フォントを作る.

    scale … ウィンドウの大きさに合わせた倍率。QLabel やボタンに付けるときだけ渡す
    (自分で絵を描く部品は、絵ごと拡大するので渡さない)
    """
    f = QFont(FONT_FAMILY)
    f.setPointSizeF(size * scale)
    f.setBold(bold)
    if spacing:
        f.setLetterSpacing(QFont.AbsoluteSpacing, spacing * scale)
    return f


def color(hex_color, alpha=255):
    c = QColor(hex_color)
    c.setAlpha(alpha)
    return c


def pen(hex_color, width=1.0, alpha=255, dashed=False):
    p = QPen(color(hex_color, alpha), width)
    p.setCapStyle(Qt.RoundCap)
    p.setJoinStyle(Qt.RoundJoin)
    if dashed:
        p.setDashPattern([2.0, 3.0])
    return p


def rgba(hex_color, alpha):
    """色と透明度を、スタイルシートに書ける形の文字列にする."""
    c = QColor(hex_color)
    return f'rgba({c.red()}, {c.green()}, {c.blue()}, {alpha})'


def arrow_head(tip: QPointF, dx: float, dy: float, size: float) -> QPolygonF:
    """先が tip にあり、(dx, dy) の向きを指す三角形."""
    norm = math.hypot(dx, dy) or 1.0
    ux, uy = dx / norm, dy / norm
    back = QPointF(tip.x() - ux * size, tip.y() - uy * size)
    half = size * 0.55
    return QPolygonF([tip,
                      QPointF(back.x() - uy * half, back.y() + ux * half),
                      QPointF(back.x() + uy * half, back.y() - ux * half)])


def draw_arrow(painter, start: QPointF, end: QPointF, hex_color, width, head=10.0, alpha=255,
               dashed=False):
    """Start から end への矢印を描く(dashed なら点線で、先は白抜き)."""
    dx, dy = end.x() - start.x(), end.y() - start.y()
    length = math.hypot(dx, dy)
    if length < 1.0:
        return
    head = min(head, length)
    # 線は矢印の先の手前で止める(先の三角と重なって太く見えないように)
    stop = QPointF(end.x() - dx / length * head * 0.8, end.y() - dy / length * head * 0.8)
    painter.setPen(pen(hex_color, width, alpha, dashed))
    painter.drawLine(start, stop)
    painter.setPen(pen(hex_color, 1.2, alpha) if dashed else Qt.NoPen)
    painter.setBrush(Qt.NoBrush if dashed else color(hex_color, alpha))
    painter.drawPolygon(arrow_head(end, dx, dy, head))
