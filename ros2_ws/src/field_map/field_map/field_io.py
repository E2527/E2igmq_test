"""
フィールドエディタ(field_editor)の、画面に依存しない部分.

画面にも ROS にも依存しないので、PC だけでテストできる(test/test_field_io.py)。

    load_document / dump_document … フィールドのファイルを、エディタで扱う形(辞書)で読み書きする
    Calibration                    … 下に敷く画像(フィールドの図)の、画素 ↔ フィールドの座標 [m]
    grid_to_shapes                 … 画像の黒いところ(マス目の True)を、box / polygon にする
"""

from collections import deque
import json
import math
import os

import yaml

from . import field as fm

# エディタで書き出す field の項目(この順番で書く)
FIELD_KEYS = ('name', 'resolution', 'x_range', 'y_range', 'border', 'lidar_height',
              'robot_height', 'step_height', 'background')
SHAPE_KEYS = ('type', 'name', 'x', 'y', 'size_x', 'size_y', 'yaw_deg', 'radius', 'points',
              'height', 'z_min', 'z_max', 'passable')


# ---------------------------------------------------------------- 読み書き

def new_document():
    """何もないフィールド."""
    return {'name': '新しいフィールド', 'resolution': 0.05, 'x_range': [-3.0, 3.0],
            'y_range': [-2.0, 2.0], 'border': 0.05}, []


def load_document(path):
    """
    フィールドのファイルを読み、(field の辞書, 図形の辞書の並び) を返す.

    gn10 の JSON(half_x / half_y、CYLINDER)は、このリポジトリの書き方(size_x / circle)に直す。
    書き方がおかしければ field.FieldError。
    """
    fm.load_field(path)                         # まず、地図にできる書き方かを確かめる
    with open(path, encoding='utf-8') as f:
        text = f.read()
    data = json.loads(text) if path.endswith('.json') else yaml.safe_load(text)
    if isinstance(data, list):
        data = {'shapes': data}
    info = dict(data.get('field') or {})
    info.setdefault('name', os.path.splitext(os.path.basename(path))[0])
    if 'x_range' not in info or 'y_range' not in info:
        field = fm.parse_field(data, info['name'])
        info['x_range'] = [round(field.x_min, 4), round(field.x_max, 4)]
        info['y_range'] = [round(field.y_min, 4), round(field.y_max, 4)]
    shapes = [normalize_shape(s) for s in data.get('shapes') or []]
    return info, [s for s in shapes if s is not None]


def normalize_shape(item):
    """図形の辞書を、エディタで扱う書き方にそろえる(通れるだけの gn10 の VISUAL_BOX は None)."""
    kind = str(item.get('type', 'box')).lower()
    if kind == 'visual_box':
        return None
    shape = {'type': {'cylinder': 'circle'}.get(kind, kind), 'name': str(
        item.get('name', item.get('comment', '')))}
    if shape['type'] in ('box', 'circle'):
        shape['x'], shape['y'] = float(item.get('x', 0.0)), float(item.get('y', 0.0))
    if shape['type'] == 'box':
        if 'half_x' in item:
            shape['size_x'], shape['size_y'] = 2 * float(item['half_x']), 2 * float(item['half_y'])
        else:
            shape['size_x'], shape['size_y'] = float(item['size_x']), float(item['size_y'])
        shape['yaw_deg'] = float(item.get('yaw_deg', 0.0))
    elif shape['type'] == 'circle':
        shape['radius'] = float(item['radius'])
    elif shape['type'] == 'polygon':
        shape['points'] = [[float(p[0]), float(p[1])] for p in item['points']]
    for key in ('z_min', 'z_max'):
        if key in item:
            shape[key] = float(item[key])
    if 'height' in item:
        shape['z_max'] = float(item['height'])
    if item.get('passable'):
        shape['passable'] = True
    return shape


def dump_document(info, shapes):
    """(field の辞書, 図形の並び) を、人が読みやすい YAML の文字列にする(図形は 1 行に 1 個)."""
    lines = ['# フィールドの形。フィールドエディタ(ros2 run field_map field_editor)で保存した。',
             '# 手で書き換えてもよい(書き方は ros2_ws/src/field_map/README.md)。保存すると地図にすぐ反映される',
             '', 'field:']
    for key in FIELD_KEYS:
        if key in info and info[key] is not None:
            lines.append(f'  {key}: {_flow(info[key])}')
    for key, value in info.items():             # 知らない項目も消さずに残す
        if key not in FIELD_KEYS and value is not None:
            lines.append(f'  {key}: {_flow(value)}')
    lines += ['', 'shapes:']
    if not shapes:
        lines[-1] = 'shapes: []'
    for shape in shapes:
        out = dict(shape)
        if out.get('z_min', 0.0) == 0.0 and 'z_max' in out:     # 床からなら height で書く
            out.pop('z_min', None)
            out['height'] = out.pop('z_max')
        if out.get('yaw_deg') == 0.0:
            out.pop('yaw_deg')
        if not out.get('name'):
            out.pop('name', None)
        keys = [k for k in SHAPE_KEYS if k in out] + [k for k in out if k not in SHAPE_KEYS]
        lines.append('  - {' + ', '.join(f'{k}: {_flow(out[k])}' for k in keys) + '}')
    return '\n'.join(lines) + '\n'


def _flow(value):
    """YAML の 1 行に書ける形(数は 0.1 mm で丸める)."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, float):
        text = f'{round(value, 4):.4f}'.rstrip('0')
        return text + '0' if text.endswith('.') else text
    if isinstance(value, int):
        return str(value)
    if isinstance(value, (list, tuple)):
        return '[' + ', '.join(_flow(v) for v in value) + ']'
    if isinstance(value, dict):
        return '{' + ', '.join(f'{k}: {_flow(v)}' for k, v in value.items()) + '}'
    text = str(value)
    plain = yaml.safe_load(text) == text if text else False
    if plain and not any(c in text for c in ':#{}[],&*!|>\'"%@`'):
        return text
    return json.dumps(text, ensure_ascii=False)


# ---------------------------------------------------------------- 画像の位置合わせ

class Calibration:
    """
    下に敷く画像の画素 (u, v) と、フィールドの座標 (x, y) [m] の対応.

    画像は v が下向き、フィールドは y が上向きなので、p = u - i v として、
    複素数で x + i y = a p + b と書く(a の大きさが縮尺、向きが回転)。
    2 つの点の対応から a と b が決まる。
    """

    def __init__(self, a, b):
        if abs(a) < 1e-12:
            raise ValueError('縮尺が 0 です(2 つの点が同じ位置)')
        self.a, self.b = complex(a), complex(b)

    @classmethod
    def from_points(cls, points):
        """[[u1, v1, x1, y1], [u2, v2, x2, y2]] から作る."""
        (u1, v1, x1, y1), (u2, v2, x2, y2) = [[float(c) for c in p] for p in points]
        p1, p2 = complex(u1, -v1), complex(u2, -v2)
        f1, f2 = complex(x1, y1), complex(x2, y2)
        if abs(p2 - p1) < 1e-9 or abs(f2 - f1) < 1e-9:
            raise ValueError('2 つの点は、離れた場所にしてください')
        a = (f2 - f1) / (p2 - p1)
        return cls(a, f1 - a * p1)

    @classmethod
    def fit(cls, width_px, height_px, x_range, y_range):
        """画像を、縦横比を変えずにフィールドの範囲のまん中に収める(位置合わせ前の仮の置き方)."""
        fw, fh = x_range[1] - x_range[0], y_range[1] - y_range[0]
        scale = min(fw / width_px, fh / height_px)
        cx, cy = (x_range[0] + x_range[1]) / 2, (y_range[0] + y_range[1]) / 2
        # 画像のまん中 (w/2, h/2) がフィールドのまん中に来るように
        return cls(scale, complex(cx, cy) - scale * complex(width_px / 2, -height_px / 2))

    @classmethod
    def from_map_yaml(cls, meta, height_px):
        """Nav2 の map_server の地図(resolution と origin)から作る。画像の左下が origin."""
        res = float(meta['resolution'])
        ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
        return cls(res, complex(ox, oy + height_px * res))

    def to_field(self, u, v):
        f = self.a * complex(u, -v) + self.b
        return f.real, f.imag

    def to_pixel(self, x, y):
        p = (complex(x, y) - self.b) / self.a
        return p.real, -p.imag

    def points(self, width_px):
        """保存用: 画像の左上と右上の 2 点の対応."""
        return [[0.0, 0.0, *map(_r4, self.to_field(0, 0))],
                [float(width_px), 0.0, *map(_r4, self.to_field(width_px, 0))]]

    @property
    def meters_per_pixel(self):
        return abs(self.a)

    @property
    def rotation_deg(self):
        return math.degrees(math.atan2(self.a.imag, self.a.real))


def _r4(v):
    return round(v, 4)


# ---------------------------------------------------------------- 画像 → 図形

def grid_to_shapes(grid, x0, y0, step, min_cells=4, box_fill=0.85, tolerance=1.2,
                   max_boxes=30, name='画像'):
    """
    マス目(grid[row][col] が True のところが物。row 0 が下)を、図形の辞書の並びにする.

    x0, y0 … マス目の左下の角の座標 [m]、step … 1 マスの大きさ [m]

    つながっている True のかたまりごとに:
        ほぼ長方形(かたまりがその外枠の box_fill 以上を占める) → box
        ほぼ円(外枠がほぼ正方形で、その π/4 くらいを占める)     → circle
        穴の少ないかたまり                                     → polygon(輪郭を少なめの角にする)
        枠や L 字の線のように、中が空いているもの               → 長方形に分ける(max_boxes まで)
    min_cells マスより小さいかたまり(文字・ごみ)は捨てる。
    """
    rows = len(grid)
    cols = len(grid[0]) if rows else 0
    seen = [[False] * cols for _ in range(rows)]
    shapes = []
    for r in range(rows):
        for c in range(cols):
            if not grid[r][c] or seen[r][c]:
                continue
            cells = _component(grid, seen, r, c)
            if len(cells) < min_cells:
                continue
            shapes.extend(_component_to_shapes(cells, x0, y0, step, box_fill, tolerance,
                                               max_boxes))
    for i, shape in enumerate(shapes, 1):
        shape['name'] = f'{name}{i}'
    return shapes


def _component(grid, seen, r0, c0):
    """(r0, c0) とつながっている(上下左右)True のマス全部."""
    rows, cols = len(grid), len(grid[0])
    queue, cells = deque([(r0, c0)]), []
    seen[r0][c0] = True
    while queue:
        r, c = queue.popleft()
        cells.append((r, c))
        for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] and not seen[nr][nc]:
                seen[nr][nc] = True
                queue.append((nr, nc))
    return cells


def _component_to_shapes(cells, x0, y0, step, box_fill, tolerance, max_boxes):
    r_min = min(r for r, _ in cells)
    r_max = max(r for r, _ in cells)
    c_min = min(c for _, c in cells)
    c_max = max(c for _, c in cells)
    area = (r_max - r_min + 1) * (c_max - c_min + 1)
    if len(cells) >= box_fill * area:
        return [_box(c_min, r_min, c_max + 1, r_max + 1, x0, y0, step)]
    w, h = c_max - c_min + 1, r_max - r_min + 1
    if abs(w - h) <= max(1, 0.12 * max(w, h)) and 0.70 <= len(cells) / area <= 0.86 \
            and min(w, h) >= 3:
        return [{'type': 'circle', 'name': '', 'x': _r4(x0 + (c_min + c_max + 1) / 2 * step),
                 'y': _r4(y0 + (r_min + r_max + 1) / 2 * step),
                 'radius': _r4(math.sqrt(len(cells) / math.pi) * step)}]
    outline = _simplify(_outline(set(cells)), tolerance)
    if len(outline) >= 3 and abs(_area(outline)) <= 1.3 * len(cells):
        return [{'type': 'polygon', 'name': '',
                 'points': [[_r4(x0 + c * step), _r4(y0 + r * step)] for c, r in outline]}]
    rects = _rectangles(set(cells))
    if len(rects) <= max_boxes:
        return [_box(c0, r0, c1, r1, x0, y0, step) for c0, r0, c1, r1 in rects]
    return [{'type': 'polygon', 'name': '',             # 分けると多すぎるときは、輪郭だけ
             'points': [[_r4(x0 + c * step), _r4(y0 + r * step)] for c, r in outline]}]


def _box(c0, r0, c1, r1, x0, y0, step):
    """マス [c0, c1) × [r0, r1) を覆う box."""
    return {'type': 'box', 'name': '', 'x': _r4(x0 + (c0 + c1) / 2 * step),
            'y': _r4(y0 + (r0 + r1) / 2 * step), 'size_x': _r4((c1 - c0) * step),
            'size_y': _r4((r1 - r0) * step), 'yaw_deg': 0.0}


def _outline(cells):
    """かたまりの外側の輪郭を、マスの角の点 (col, row) の並びで返す(穴は無視)."""
    # かたまりの境目の辺を集めて、つなげて輪にし、いちばん長い輪(外側)を使う
    edges = {}
    for r, c in cells:
        sides = (((-1, 0), (c, r), (c + 1, r)), ((1, 0), (c + 1, r + 1), (c, r + 1)),
                 ((0, -1), (c, r + 1), (c, r)), ((0, 1), (c + 1, r), (c + 1, r + 1)))
        for (dr, dc), a, b in sides:
            if (r + dr, c + dc) not in cells:
                edges.setdefault(a, []).append(b)
    loops = []
    while edges:
        start = next(iter(edges))
        loop, point = [start], start
        while True:
            nxt = edges[point].pop()
            if not edges[point]:
                del edges[point]
            if nxt == start:
                break
            loop.append(nxt)
            point = nxt
            if point not in edges:
                break
        loops.append(loop)
    outer = max(loops, key=lambda lp: abs(_area(lp)))
    # 一直線に並んだ点を省く
    return [p for i, p in enumerate(outer)
            if _cross(outer[i - 1], p, outer[(i + 1) % len(outer)]) != 0]


def _simplify(points, tolerance):
    """輪の点を減らす(ダグラス・ピーカー法。tolerance マスより小さいでこぼこを無視)."""
    if len(points) <= 4:
        return points
    far = max(range(len(points)), key=lambda i: _dist2(points[0], points[i]))
    first = _dp(points[:far + 1], tolerance)
    second = _dp(points[far:] + [points[0]], tolerance)
    return first[:-1] + second[:-1]


def _dp(points, tolerance):
    if len(points) <= 2:
        return points
    a, b = points[0], points[-1]
    index, dmax = 0, -1.0
    for i in range(1, len(points) - 1):
        d = _line_distance(points[i], a, b)
        if d > dmax:
            index, dmax = i, d
    if dmax <= tolerance:
        return [a, b]
    return _dp(points[:index + 1], tolerance)[:-1] + _dp(points[index:], tolerance)


def _rectangles(cells):
    """かたまりを長方形 (c0, r0, c1, r1) に分ける(横の並びを、同じ幅で続く行どうしで縦につなぐ)."""
    by_row = {}
    for r, c in cells:
        by_row.setdefault(r, []).append(c)
    rects, open_rects, prev = [], {}, None
    for r in sorted(by_row):
        cs = sorted(by_row[r])
        row_runs, start = [], cs[0]
        for a, b in zip(cs, cs[1:] + [None]):
            if b != a + 1:
                row_runs.append((start, a + 1))
                start = b
        still_open = {}
        for run in row_runs:
            if run in open_rects and prev == r - 1:
                still_open[run] = open_rects.pop(run)
            else:
                still_open[run] = r
        rects += [(c0, r0, c1, prev + 1) for (c0, c1), r0 in open_rects.items()]
        open_rects, prev = still_open, r
    rects += [(c0, r0, c1, prev + 1) for (c0, c1), r0 in open_rects.items()]
    return rects


def _area(points):
    return sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(points, points[1:] + points[:1])) / 2


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])


def _dist2(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def _line_distance(p, a, b):
    length = math.sqrt(_dist2(a, b))
    if length == 0:
        return math.sqrt(_dist2(p, a))
    return abs((b[0] - a[0]) * (a[1] - p[1]) - (a[0] - p[0]) * (b[1] - a[1])) / length
