import json
import os

from field_map import field as fm
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FIELDS = os.path.join(HERE, '..', 'fields')


def make(shapes, x=(-1.0, 1.0), y=(-1.0, 1.0), resolution=0.1, border=0.0):
    data = {'field': {'x_range': list(x), 'y_range': list(y), 'resolution': resolution,
                      'border': border}, 'shapes': shapes}
    return fm.parse_field(data)


def occupied(field, cells, x, y):
    """座標 (x, y) のマスがふさがっているか."""
    col = int((x - field.x_min) / field.resolution)
    row = int((y - field.y_min) / field.resolution)
    return cells[row * field.width + col] == fm.OCCUPIED


def test_empty_field_is_free_and_has_the_right_size():
    field = make([], x=(-1.0, 1.0), y=(0.0, 0.5))
    cells = fm.rasterize(field)
    assert (field.width, field.height) == (20, 5)
    assert cells == [fm.FREE] * 100


def test_border_walls():
    field = make([], border=0.1)
    cells = fm.rasterize(field)
    assert occupied(field, cells, -0.95, 0.0) and occupied(field, cells, 0.0, 0.95)
    assert not occupied(field, cells, -0.85, 0.0) and not occupied(field, cells, 0.0, 0.0)


def test_box_marks_only_its_cells():
    field = make([{'type': 'box', 'x': 0.25, 'y': -0.25, 'size_x': 0.2, 'size_y': 0.2}])
    cells = fm.rasterize(field)
    assert occupied(field, cells, 0.25, -0.25)
    assert not occupied(field, cells, -0.5, 0.5) and not occupied(field, cells, 0.55, -0.25)


def test_thin_wall_is_not_lost():
    # マス(10 cm)より薄い 2 cm の壁も、地図からぬけない
    field = make([{'type': 'box', 'x': 0.03, 'y': 0.0, 'size_x': 0.02, 'size_y': 1.0}])
    cells = fm.rasterize(field)
    assert occupied(field, cells, 0.05, 0.0)


def test_rotated_box():
    # 45° 回した細長い箱は、ななめの線になる
    field = make([{'type': 'box', 'x': 0.0, 'y': 0.0, 'size_x': 1.6, 'size_y': 0.05,
                   'yaw_deg': 45}])
    cells = fm.rasterize(field)
    assert occupied(field, cells, 0.45, 0.45) and occupied(field, cells, -0.45, -0.45)
    assert not occupied(field, cells, 0.45, -0.45) and not occupied(field, cells, -0.45, 0.45)


def test_circle_and_polygon():
    field = make([{'type': 'circle', 'x': -0.5, 'y': 0.5, 'radius': 0.15},
                  {'type': 'polygon', 'points': [[0.2, -0.9], [0.9, -0.9], [0.9, -0.2]]}])
    cells = fm.rasterize(field)
    assert occupied(field, cells, -0.5, 0.5) and not occupied(field, cells, -0.5, 0.0)
    assert occupied(field, cells, 0.8, -0.8) and not occupied(field, cells, 0.25, -0.35)


def test_passable_shapes_are_skipped():
    field = make([{'type': 'box', 'x': 0, 'y': 0, 'size_x': 0.5, 'size_y': 0.5,
                   'passable': True}])
    assert fm.rasterize(field) == [fm.FREE] * (field.width * field.height)


def test_shapes_outside_the_field_are_clipped():
    field = make([{'type': 'circle', 'x': 5.0, 'y': 5.0, 'radius': 0.5},
                  {'type': 'box', 'x': 1.0, 'y': 0.0, 'size_x': 0.4, 'size_y': 0.4}])
    cells = fm.rasterize(field)
    assert len(cells) == field.width * field.height
    assert occupied(field, cells, 0.95, 0.0)


def test_gn10_json_format(tmp_path):
    # ほかの高専の gn10-pointcloud-localization の JSON(図形の並びだけ)も読める
    path = tmp_path / 'gn10.json'
    path.write_text(json.dumps([
        {'type': 'VISUAL_BOX', 'x': 0, 'y': 0, 'half_x': 2, 'half_y': 2, 'passable': True},
        {'type': 'BOX', 'x': 0.0, 'y': 1.0, 'half_x': 1.0, 'half_y': 0.012, 'cost': 120},
        {'type': 'CYLINDER', 'x': 0.5, 'y': -0.5, 'radius': 0.14, 'z_min': 0, 'z_max': 0.3},
    ]), encoding='utf-8')
    field = fm.load_field(str(path))
    assert len(field.shapes) == 2 and field.name == 'gn10'
    # 広さを書いていないときは、図形が全部入る広さ(まわりに 0.5 m)
    assert field.x_min == pytest.approx(-1.5) and field.y_max == pytest.approx(1.512)
    cells = fm.rasterize(field)
    assert occupied(field, cells, 0.0, 1.0) and occupied(field, cells, 0.5, -0.5)


@pytest.mark.parametrize('data', [
    {'field': {'x_range': [1, -1], 'y_range': [-1, 1]}},                     # 逆
    {'field': {'x_range': [-1, 1], 'y_range': [-1, 1], 'resolution': 0}},
    {'field': {'x_range': [-1, 1], 'y_range': [-1, 1]}, 'shapes': [{'type': 'star'}]},
    {'field': {'x_range': [-1, 1], 'y_range': [-1, 1]},
     'shapes': [{'type': 'box', 'x': 0, 'y': 0, 'size_x': -1, 'size_y': 1}]},
    {'field': {'x_range': [-1, 1], 'y_range': [-1, 1]},
     'shapes': [{'type': 'polygon', 'points': [[0, 0], [1, 1]]}]},
    {'field': {}},                                                            # 広さも図形もない
    'ただの文字',
])
def test_bad_files_give_a_clear_error(data):
    with pytest.raises(fm.FieldError):
        fm.parse_field(data)


def test_unreadable_file(tmp_path):
    with pytest.raises(fm.FieldError):
        fm.load_field(str(tmp_path / 'none.yaml'))
    broken = tmp_path / 'broken.yaml'
    broken.write_text('field: [', encoding='utf-8')
    with pytest.raises(fm.FieldError):
        fm.load_field(str(broken))


def test_write_map_files(tmp_path):
    field = make([{'type': 'box', 'x': -0.95, 'y': 0.95, 'size_x': 0.05, 'size_y': 0.05}],
                 x=(-1.0, 1.0), y=(0.0, 1.0))
    cells = fm.rasterize(field)
    pgm, meta = fm.write_map_files(field, cells, str(tmp_path / 'out' / 'field'))
    data = open(pgm, 'rb').read()
    header = b'P5\n20 10\n255\n'
    assert data.startswith(header) and len(data) == len(header) + 200
    pixels = data[len(header):]
    assert pixels[0] == 0 and pixels[-1] == 254       # 左上(-x, +y)が黒、右下が白
    text = open(meta, encoding='utf-8').read()
    assert 'image: field.pgm' in text and 'origin:' in text and 'mode: trinary' in text


def test_preview_shows_the_shapes():
    field = make([], border=0.1)
    lines = fm.preview(field, fm.rasterize(field), columns=10).split('\n')
    assert lines[0] == '#' * 10 and lines[-1] == '#' * 10
    assert set(lines[len(lines) // 2]) == {'#', '.'}


def test_example_field_is_valid():
    field = fm.load_field(os.path.join(FIELDS, 'example.yaml'))
    cells = fm.rasterize(field)
    assert field.shapes and fm.OCCUPIED in cells and fm.FREE in cells


def test_resolve_field_path(tmp_path):
    (tmp_path / 'a.yaml').write_text('', encoding='utf-8')
    (tmp_path / 'b.json').write_text('', encoding='utf-8')
    assert fm.resolve_field_path('a', [str(tmp_path)]) == str(tmp_path / 'a.yaml')
    assert fm.resolve_field_path('b', [str(tmp_path)]) == str(tmp_path / 'b.json')
    assert fm.resolve_field_path('/x/y.yaml') == '/x/y.yaml'          # パスはそのまま
    # 見つからなければ、最後の候補を返す(エラーの知らせに場所が出るように)
    assert fm.resolve_field_path('none', [str(tmp_path)]).startswith(str(tmp_path / 'none'))
