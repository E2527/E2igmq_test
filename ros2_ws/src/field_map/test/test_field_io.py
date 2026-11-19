import math

from field_map import field as fm
from field_map import field_io as fio
import pytest


def grid_from(text):
    """文字の絵(# が物、上の行が +y)からマス目を作る(row 0 が下)."""
    rows = [line for line in text.strip('\n').split('\n')]
    return [[ch == '#' for ch in row] for row in reversed(rows)]


def test_solid_rectangle_becomes_one_box():
    grid = grid_from("""
........
..####..
..####..
........
""")
    shapes = fio.grid_to_shapes(grid, x0=0.0, y0=0.0, step=0.1)
    assert len(shapes) == 1 and shapes[0]['type'] == 'box'
    s = shapes[0]
    assert (s['x'], s['y'], s['size_x'], s['size_y']) == (0.4, 0.2, 0.4, 0.2)
    assert s['name'] == '画像1'


def test_small_specks_are_dropped():
    grid = grid_from("""
#.......
........
.....##.
.....##.
""")
    shapes = fio.grid_to_shapes(grid, 0.0, 0.0, 0.1, min_cells=4)
    assert len(shapes) == 1


def test_frame_becomes_boxes_not_a_filled_polygon():
    # 中が空いた枠(フィールドの外枠の線など)は、中を埋めずに 4 本の box にする
    grid = grid_from("""
##########
#........#
#........#
#........#
##########
""")
    shapes = fio.grid_to_shapes(grid, 0.0, 0.0, 0.1)
    assert all(s['type'] == 'box' for s in shapes) and 2 <= len(shapes) <= 4
    field = fm.parse_field({'field': {'x_range': [0, 1], 'y_range': [0, 0.5], 'resolution': 0.1,
                                      'border': 0}, 'shapes': shapes})
    cells = fm.rasterize(field)
    assert cells[2 * 10 + 5] == fm.FREE                     # まん中は通れる
    assert cells[0] == fm.OCCUPIED and cells[4 * 10 + 9] == fm.OCCUPIED


def test_triangle_becomes_polygon():
    grid = grid_from("""
#.........
##........
####......
######....
########..
##########
""")
    shapes = fio.grid_to_shapes(grid, 0.0, 0.0, 0.1)
    assert len(shapes) == 1 and shapes[0]['type'] == 'polygon'
    assert 3 <= len(shapes[0]['points']) <= 8


def test_disc_becomes_circle():
    grid = grid_from("""
....###....
..#######..
.#########.
.#########.
###########
###########
###########
.#########.
.#########.
..#######..
....###....
""")
    shapes = fio.grid_to_shapes(grid, 0.0, 0.0, 0.1)
    assert len(shapes) == 1 and shapes[0]['type'] == 'circle'
    assert (shapes[0]['x'], shapes[0]['y']) == pytest.approx((0.55, 0.55))
    assert shapes[0]['radius'] == pytest.approx(0.55, abs=0.05)


def test_l_shape_line_is_split_into_boxes():
    grid = grid_from("""
##........
##........
##........
##########
##########
""")
    shapes = fio.grid_to_shapes(grid, 0.0, 0.0, 0.1)
    assert sum(s['size_x'] * s['size_y'] for s in shapes if s['type'] == 'box') \
        == pytest.approx(0.26) or shapes[0]['type'] == 'polygon'


def test_calibration_from_two_points():
    # 画像の (100, 200) が (0, 0)、(300, 200) が (2, 0) → 1 画素 1 cm、回転なし
    cal = fio.Calibration.from_points([[100, 200, 0.0, 0.0], [300, 200, 2.0, 0.0]])
    assert cal.meters_per_pixel == pytest.approx(0.01)
    assert cal.rotation_deg == pytest.approx(0.0)
    x, y = cal.to_field(100, 100)                           # 画像の上 = フィールドの +y
    assert (x, y) == pytest.approx((0.0, 1.0))
    assert cal.to_pixel(*cal.to_field(37, 81)) == pytest.approx((37, 81))
    again = fio.Calibration.from_points(cal.points(400))
    assert again.to_field(250, 50) == pytest.approx(cal.to_field(250, 50))


def test_calibration_with_rotation():
    # 画像の右向きがフィールドの +y(90° 回っている)
    cal = fio.Calibration.from_points([[0, 0, 0.0, 0.0], [100, 0, 0.0, 1.0]])
    assert cal.rotation_deg == pytest.approx(90.0)
    assert cal.to_field(0, 100) == pytest.approx((1.0, 0.0))


def test_calibration_errors():
    with pytest.raises(ValueError):
        fio.Calibration.from_points([[10, 10, 0, 0], [10, 10, 1, 1]])
    with pytest.raises(ValueError):
        fio.Calibration.from_points([[10, 10, 1, 1], [20, 20, 1, 1]])


def test_calibration_fit_and_map_yaml():
    fit = fio.Calibration.fit(600, 400, [-3.0, 3.0], [-2.0, 2.0])
    assert fit.to_field(0, 0) == pytest.approx((-3.0, 2.0))
    assert fit.to_field(600, 400) == pytest.approx((3.0, -2.0))
    meta = {'resolution': 0.05, 'origin': [-1.0, -2.0, 0.0]}
    cal = fio.Calibration.from_map_yaml(meta, height_px=80)
    assert cal.to_field(0, 80) == pytest.approx((-1.0, -2.0))      # 左下が origin
    assert cal.to_field(20, 0) == pytest.approx((0.0, 2.0))


def test_dump_and_load_round_trip(tmp_path):
    info = {'name': 'テスト: 2027', 'resolution': 0.05, 'x_range': [-1.0, 1.0],
            'y_range': [-1.0, 1.0], 'border': 0.05, 'lidar_height': 0.2,
            'background': {'image': 'field.png', 'opacity': 0.5,
                           'points': [[0.0, 0.0, -1.0, 1.0], [100.0, 0.0, 1.0, 1.0]]}}
    shapes = [{'type': 'box', 'name': '台', 'x': 0.1, 'y': -0.2, 'size_x': 0.3, 'size_y': 0.4,
               'yaw_deg': 15.0, 'z_max': 0.25},
              {'type': 'circle', 'name': '', 'x': 0.5, 'y': 0.5, 'radius': 0.1},
              {'type': 'polygon', 'name': '棚', 'points': [[0, 0], [0.2, 0], [0.2, 0.2]],
               'z_min': 0.7, 'z_max': 0.8}]
    text = fio.dump_document(info, shapes)
    assert '  - {type: box, name: 台, x: 0.1, y: -0.2' in text and 'height: 0.25' in text
    path = tmp_path / 'f.yaml'
    path.write_text(text, encoding='utf-8')
    info2, shapes2 = fio.load_document(str(path))
    assert info2['name'] == 'テスト: 2027' and info2['background']['opacity'] == 0.5
    assert shapes2[0]['z_max'] == 0.25 and shapes2[0]['yaw_deg'] == 15.0
    assert shapes2[1]['radius'] == 0.1 and shapes2[2]['z_min'] == 0.7
    field = fm.load_field(str(path))                       # 地図にもできる
    assert len(field.shapes) == 3 and field.lidar_heights == (0.2,)


def test_load_gn10_json(tmp_path):
    path = tmp_path / 'g.json'
    path.write_text('[{"type": "VISUAL_BOX", "x": 0, "y": 0, "half_x": 1, "half_y": 1},'
                    ' {"type": "BOX", "x": 0, "y": 0.5, "half_x": 0.5, "half_y": 0.01,'
                    ' "z_min": 0, "z_max": 0.15, "comment": "壁"},'
                    ' {"type": "CYLINDER", "x": 0.2, "y": -0.2, "radius": 0.1}]',
                    encoding='utf-8')
    info, shapes = fio.load_document(str(path))
    assert [s['type'] for s in shapes] == ['box', 'circle']
    assert shapes[0]['size_x'] == 1.0 and shapes[0]['name'] == '壁'
    assert 'x_range' in info and 'y_range' in info


def test_new_document_is_valid():
    info, shapes = fio.new_document()
    field = fm.parse_field({'field': info, 'shapes': shapes})
    assert field.width > 0 and not field.shapes


def test_flow_quotes_tricky_names():
    assert fio._flow('台') == '台'
    assert fio._flow('a: b') == '"a: b"'
    assert fio._flow('yes') == '"yes"'                     # YAML で true になってしまう
    assert fio._flow(0.30000001) == '0.3' and fio._flow(2.0) == '2.0'
    assert not math.isnan(float(fio._flow(1e-9)))
