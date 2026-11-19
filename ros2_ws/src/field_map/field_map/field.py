"""
フィールドの形を書いたファイルから、地図(マス目)を作る.

ROS に依存しないので、PC だけでテストできる(test/test_field.py)。

    フィールドのファイル(YAML)… fields/*.yaml。壁や台を「箱」「円」「多角形」で書く
    地図                        … マス目ごとに「通れる(0)」か「ふさがっている(100)」か。
                                   ROS の nav_msgs/OccupancyGrid と同じ並び(左下から右へ、下の行から上へ)

高さ(z_min / z_max)から、地図を 2 種類作る:
    nav   … 通れるか用。ロボットの高さ(床〜robot_height)にかかる物。経路計画が使う
    lidar … 位置合わせ用。2D LiDAR の高さ(lidar_height)にかかる物だけ。AMCL が使う
            (低い台や上の棚は LiDAR に映らないので、入れると位置がずれる)

座標の決まり(ROS と同じ): x が前(フィールドの図の右)、y が左(図の上)、単位は m、角度は度。

ほかの高専の gn10-pointcloud-localization(Apache-2.0)の、フィールドを JSON で書く方法を参考にした。
その JSON(BOX / CYLINDER / VISUAL_BOX の並び)も、そのまま読める。
"""

from dataclasses import dataclass, field as dataclass_field
import json
import math
import os

import yaml

FREE = 0
OCCUPIED = 100
NAV = 'nav'         # 通れるか用の地図
LIDAR = 'lidar'     # 位置合わせ用の地図


class FieldError(ValueError):
    """フィールドのファイルの書き方がおかしい."""


@dataclass
class Shape:
    """ふさがっている場所 1 つ。kind は box / circle / polygon."""

    kind: str
    points: list = dataclass_field(default_factory=list)   # box と polygon の角 [(x, y), …]
    center: tuple = (0.0, 0.0)                              # circle の中心
    radius: float = 0.0                                     # circle の半径
    name: str = ''
    z_min: float = 0.0                                      # 下の高さ [m](床なら 0)
    z_max: float = math.inf                                 # 上の高さ [m](書かなければ天井まで)

    def bounds(self):
        """(x_min, x_max, y_min, y_max)."""
        if self.kind == 'circle':
            cx, cy = self.center
            return cx - self.radius, cx + self.radius, cy - self.radius, cy + self.radius
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        return min(xs), max(xs), min(ys), max(ys)


@dataclass
class Field:
    """フィールド 1 枚分."""

    name: str
    resolution: float           # 1 マスの大きさ [m]
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    border: float               # まわりの壁の厚さ [m](0 なら付けない)
    shapes: list
    lidar_heights: tuple = ()   # 2D LiDAR の高さ [m](なければ、全部の図形が映るとみなす)
    robot_height: float = math.inf  # ロボットの高さ [m]。これより上の物は下をくぐれる
    step_height: float = 0.0    # 乗り越えられる高さ [m]。これ以下の物は通れる

    def blocks(self, shape, layer):
        """図形が、その地図(NAV か LIDAR)でふさがっている場所になるか."""
        if layer == LIDAR:
            if not self.lidar_heights:
                return True
            return any(shape.z_min <= h <= shape.z_max for h in self.lidar_heights)
        return shape.z_max > self.step_height and shape.z_min < self.robot_height

    @property
    def width(self):
        """横(x)のマスの数."""
        return max(1, int(math.ceil((self.x_max - self.x_min) / self.resolution - 1e-9)))

    @property
    def height(self):
        """縦(y)のマスの数."""
        return max(1, int(math.ceil((self.y_max - self.y_min) / self.resolution - 1e-9)))


# ---- 読み込み ----

def load_field(path):
    """フィールドのファイル(.yaml か .json)を読む。おかしいところがあれば FieldError."""
    try:
        with open(path, encoding='utf-8') as f:
            text = f.read()
        data = json.loads(text) if path.endswith('.json') else yaml.safe_load(text)
    except (OSError, ValueError, yaml.YAMLError) as e:
        raise FieldError(f'{path} を読めません: {e}')
    name = os.path.splitext(os.path.basename(path))[0]
    return parse_field(data, name)


def parse_field(data, name='field'):
    """読み込んだ中身(辞書、または gn10 の JSON の並び)から Field を作る."""
    if isinstance(data, list):                  # gn10 の JSON: 図形の並びだけ
        data = {'shapes': data}
    if not isinstance(data, dict):
        raise FieldError('一番上は「field:」と「shapes:」にしてください')
    info = data.get('field') or {}
    shapes = []
    for i, item in enumerate(data.get('shapes') or []):
        try:
            shape = _parse_shape(item)
        except (KeyError, TypeError, ValueError) as e:
            raise FieldError(f'shapes の {i + 1} 個目がおかしい({e}): {item}')
        if shape is not None:
            shapes.append(shape)
    try:
        resolution = float(info.get('resolution', 0.05))
        border = float(info.get('border', resolution))
        if 'x_range' in info or 'y_range' in info:
            x_min, x_max = (float(v) for v in info['x_range'])
            y_min, y_max = (float(v) for v in info['y_range'])
        else:                                   # 広さを書いていなければ、図形が全部入る広さ
            if not shapes:
                raise FieldError('x_range / y_range も図形もありません')
            boxes = [s.bounds() for s in shapes]
            margin = float(info.get('margin', 0.5))
            x_min = min(b[0] for b in boxes) - margin
            x_max = max(b[1] for b in boxes) + margin
            y_min = min(b[2] for b in boxes) - margin
            y_max = max(b[3] for b in boxes) + margin
    except (KeyError, TypeError, ValueError) as e:
        raise FieldError(f'field の書き方がおかしい: {e}')
    if not 0.005 <= resolution <= 1.0:
        raise FieldError(f'resolution は 0.005〜1.0 m にしてください({resolution})')
    if x_max <= x_min or y_max <= y_min:
        raise FieldError('x_range / y_range は [小さいほう, 大きいほう] の順にしてください')
    if border < 0:
        raise FieldError('border は 0 以上にしてください')
    try:
        lidar = info.get('lidar_height')
        lidar_heights = tuple(float(v) for v in (lidar if isinstance(lidar, list) else
                                                 ([] if lidar is None else [lidar])))
        robot_height = float(info.get('robot_height', math.inf))
        step_height = float(info.get('step_height', 0.0))
    except (TypeError, ValueError) as e:
        raise FieldError(f'lidar_height / robot_height / step_height の書き方がおかしい: {e}')
    if robot_height <= 0 or step_height < 0 or any(h < 0 for h in lidar_heights):
        raise FieldError('robot_height は正、step_height と lidar_height は 0 以上にしてください')
    if step_height >= robot_height:
        raise FieldError('step_height は robot_height より小さくしてください')
    result = Field(str(info.get('name', name)), resolution, x_min, x_max, y_min, y_max,
                   border, shapes, lidar_heights, robot_height, step_height)
    if result.width * result.height > 4000 * 4000:
        raise FieldError('地図が大きすぎます(resolution を大きくしてください)')
    return result


def _parse_shape(item):
    """図形 1 つ。通れるもの(passable: true、gn10 の VISUAL_BOX)は None."""
    kind = str(item['type']).lower()
    if item.get('passable') or kind == 'visual_box':
        return None
    name = str(item.get('name', item.get('comment', '')))
    x, y = float(item.get('x', 0.0)), float(item.get('y', 0.0))
    shape = _parse_outline(kind, item, x, y, name)
    # 高さ: z_min / z_max(gn10 と同じ)、または height(床からの高さ。z_max と同じ)
    if 'height' in item and 'z_max' in item:
        raise ValueError('height と z_max は、どちらか片方だけ書いてください')
    shape.z_min = float(item.get('z_min', 0.0))
    shape.z_max = float(item.get('z_max', item.get('height', math.inf)))
    if shape.z_max <= shape.z_min:
        raise ValueError('z_max(height)は z_min より大きくしてください')
    return shape


def _parse_outline(kind, item, x, y, name):
    """図形の上から見た形."""
    if kind == 'box':
        if 'half_x' in item:                    # gn10 の書き方
            hx, hy = float(item['half_x']), float(item['half_y'])
        else:
            hx, hy = float(item['size_x']) / 2.0, float(item['size_y']) / 2.0
        if hx <= 0 or hy <= 0:
            raise ValueError('大きさは正の値にしてください')
        yaw = math.radians(float(item.get('yaw_deg', 0.0)))
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(x + c * dx - s * dy, y + s * dx + c * dy)
                   for dx, dy in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]
        return Shape('box', points=corners, name=name)
    if kind in ('circle', 'cylinder'):
        radius = float(item['radius'])
        if radius <= 0:
            raise ValueError('radius は正の値にしてください')
        return Shape('circle', center=(x, y), radius=radius, name=name)
    if kind == 'polygon':
        points = [(float(p[0]), float(p[1])) for p in item['points']]
        if len(points) < 3:
            raise ValueError('points は 3 個以上にしてください')
        return Shape('polygon', points=points, name=name)
    raise ValueError(f'type は box / circle / polygon のどれかにしてください({kind})')


SOURCE_FIELDS = os.path.expanduser('~/ros2_ws/src/field_map/fields')


def resolve_field_path(name, folders=None):
    """
    「example」のような名前なら fields/ の中を探す。パスならそのまま.

    先にソースの fields/(dev コンテナの ~/ros2_ws/src/field_map/fields)を探すので、
    新しく作ったファイルも colcon build しなくてすぐ使える。なければ、インストールした場所を探す。
    """
    if os.sep in name or name.endswith(('.yaml', '.yml', '.json')):
        return os.path.abspath(os.path.expanduser(name))
    if folders is None:
        folders = [SOURCE_FIELDS]
        try:        # ROS がない PC でも、この関数より上は使えるように、ここで読み込む
            from ament_index_python.packages import get_package_share_directory
            folders.append(os.path.join(get_package_share_directory('field_map'), 'fields'))
        except (ImportError, LookupError):
            pass
    candidates = [os.path.join(folder, name + ext) for folder in folders
                  for ext in ('.yaml', '.yml', '.json')]
    found = next((c for c in candidates if os.path.isfile(c)), None)
    if found is None and folders:       # 見つからないときは、エラーの知らせに出す場所を返す
        found = os.path.join(folders[-1], name + '.yaml')
    return found or name


# ---- 地図(マス目)にする ----

def rasterize(field, layer=NAV):
    """
    地図を作る。マスの値の並び(長さ width × height)を返す.

    layer … NAV(通れるか用)か LIDAR(位置合わせ用)。まわりの壁は、どちらにも入る。

    少しでも図形にかかるマスは、ふさがっている(OCCUPIED)にする。
    マスより細い壁(2 cm の板など)も、ぬけずに地図に入る。
    """
    w, h, res = field.width, field.height, field.resolution
    cells = [FREE] * (w * h)
    for shape in field.shapes:
        if not field.blocks(shape, layer):
            continue
        x0, x1, y0, y1 = shape.bounds()
        col0 = max(0, int(math.floor((x0 - field.x_min) / res)))
        col1 = min(w - 1, int(math.floor((x1 - field.x_min) / res)))
        row0 = max(0, int(math.floor((y0 - field.y_min) / res)))
        row1 = min(h - 1, int(math.floor((y1 - field.y_min) / res)))
        for row in range(row0, row1 + 1):
            cy0 = field.y_min + row * res
            for col in range(col0, col1 + 1):
                cx0 = field.x_min + col * res
                if _touches(shape, cx0, cy0, cx0 + res, cy0 + res):
                    cells[row * w + col] = OCCUPIED
    edge = int(math.ceil(field.border / res - 1e-9))
    if edge > 0:
        for row in range(h):
            for col in range(w):
                if min(row, col, h - 1 - row, w - 1 - col) < edge:
                    cells[row * w + col] = OCCUPIED
    return cells


def _touches(shape, x0, y0, x1, y1):
    """図形が、四角いマス [x0, x1] × [y0, y1] に少しでもかかるか."""
    if shape.kind == 'circle':
        cx, cy = shape.center
        nx, ny = min(max(cx, x0), x1), min(max(cy, y0), y1)    # マスの中で中心にいちばん近い点
        return (nx - cx) ** 2 + (ny - cy) ** 2 <= shape.radius ** 2
    return _polygon_touches_rect(shape.points, x0, y0, x1, y1)


def _polygon_touches_rect(points, x0, y0, x1, y1):
    """多角形がマスにかかるか: 角がマスの中 / マスの角が多角形の中 / 辺どうしが交わる."""
    if any(x0 <= px <= x1 and y0 <= py <= y1 for px, py in points):
        return True
    corners = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    if any(_inside(points, cx, cy) for cx, cy in corners):
        return True
    rect_edges = list(zip(corners, corners[1:] + corners[:1]))
    for a, b in zip(points, points[1:] + points[:1]):
        if any(_segments_cross(a, b, c, d) for c, d in rect_edges):
            return True
    return False


def _inside(points, x, y):
    """点 (x, y) が多角形の中か(右に向かう線が、辺と何回交わるか)."""
    inside = False
    for (ax, ay), (bx, by) in zip(points, points[1:] + points[:1]):
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def _segments_cross(a, b, c, d):
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    d1, d2 = side(c, d, a), side(c, d, b)
    d3, d4 = side(a, b, c), side(a, b, d)
    return d1 * d2 <= 0 and d3 * d4 <= 0 and (d1, d2, d3, d4) != (0, 0, 0, 0)


# ---- 書き出し・確認 ----

def write_map_files(field, cells, prefix):
    """
    Nav2 の map_server で読める地図のファイル(prefix.pgm と prefix.yaml)を書く.

    PGM は白が通れる、黒がふさがっている(画像の上がフィールドの +y)。
    """
    w, h = field.width, field.height
    pixels = bytearray()
    for row in range(h - 1, -1, -1):            # 画像は上の行から
        pixels.extend(0 if v == OCCUPIED else 254 for v in cells[row * w:(row + 1) * w])
    folder = os.path.dirname(os.path.abspath(prefix))
    os.makedirs(folder, exist_ok=True)
    with open(prefix + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h) + bytes(pixels))
    meta = {'image': os.path.basename(prefix) + '.pgm', 'mode': 'trinary',
            'resolution': field.resolution, 'origin': [field.x_min, field.y_min, 0.0],
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}
    with open(prefix + '.yaml', 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f, sort_keys=False, allow_unicode=True)
    return prefix + '.pgm', prefix + '.yaml'


PREVIEW_LEGEND = ('# 通れない・LiDAR に映る   + 通れないが LiDAR に映らない(低い台など)   '
                  '^ LiDAR に映るが下をくぐれる   . 通れる')


def preview(field, cells, lidar_cells=None, columns=60):
    """
    ターミナルで地図をざっと見るための文字の絵(上が +y).

    lidar_cells(位置合わせ用の地図)も渡すと、PREVIEW_LEGEND の 4 種類の文字で描き分ける。
    """
    w, h = field.width, field.height
    step = max(1, int(math.ceil(w / columns)))
    lidar_cells = cells if lidar_cells is None else lidar_cells
    lines = []
    for top in range(h - 1, -1, -2 * step):     # 文字は縦長なので、縦は 2 倍まとめる
        line = []
        for left in range(0, w, step):
            block = [r * w + c
                     for r in range(max(0, top - 2 * step + 1), top + 1)
                     for c in range(left, min(w, left + step))]
            nav = any(cells[i] == OCCUPIED for i in block)
            seen = any(lidar_cells[i] == OCCUPIED for i in block)
            line.append('#' if nav and seen else '+' if nav else '^' if seen else '.')
        lines.append(''.join(line))
    return '\n'.join(lines)
