import os

import pytest
from swerve_drive import setup_files as sf

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, '..', '..', '..', '..')
APP_CONFIG = os.path.join(REPO, 'firmware', 'motor_node_f303k8', 'Core', 'Inc', 'app_config.h')
SWERVE_YAML = os.path.join(HERE, '..', 'config', 'swerve.yaml')

HEADER = """/* 説明 */
#define APP_COUNTS_PER_REV 1440.0f
#define APP_SWERVE_STEER_RATIO 0.25f        /* 同じ向き */
#define APP_SWERVE_WHEEL_RATIO 0.5f         /* 逆向き */
#define APP_SWERVE_COUPLING 0.0f
#define APP_SWERVE_MAX_MOTOR_SPEED 25.0f    /* 上限 */
#define APP_SWERVE_ANGLE_KP 8.0f
"""

YAML = """swerve_drive:
  ros__parameters:
    module_names: [front_left, front_right, rear_left, rear_right]
    device_ids:   [1, 2, 3, 4]              # ID
    module_x:     [0.25, 0.25, -0.25, -0.25]  # 前
    module_y:     [0.25, -0.25, 0.25, -0.25]  # 左
    wheel_radius: 0.05                      # 半径
    max_wheel_speed: 12.0
fake_modules:
  ros__parameters:
    steer_speed: 6.0
"""


def test_defines_roundtrip_keeps_comments_and_other_lines():
    values = sf.read_defines(HEADER)
    assert values['APP_COUNTS_PER_REV'] == 1440.0 and values['APP_SWERVE_STEER_RATIO'] == 0.25
    text = sf.write_defines(HEADER, {'APP_COUNTS_PER_REV': 3200, 'APP_SWERVE_STEER_RATIO': -0.125})
    assert '#define APP_COUNTS_PER_REV 3200.0f\n' in text
    assert '#define APP_SWERVE_STEER_RATIO -0.125f        /* 同じ向き */' in text
    assert '#define APP_SWERVE_ANGLE_KP 8.0f' in text and text.startswith('/* 説明 */')
    assert sf.read_defines(text)['APP_SWERVE_STEER_RATIO'] == -0.125


def test_defines_missing_name():
    with pytest.raises(KeyError):
        sf.read_defines('#define OTHER 1.0f\n')
    with pytest.raises(KeyError):
        sf.write_defines('#define OTHER 1.0f\n', {'APP_COUNTS_PER_REV': 1})


def test_yaml_roundtrip_keeps_comments_and_other_sections():
    values = sf.read_yaml_values(YAML)
    assert values['module_x'] == [0.25, 0.25, -0.25, -0.25] and values['wheel_radius'] == 0.05
    _, _, x, y = sf.module_layout(4, 0.6, 0.4)
    text = sf.write_yaml_values(YAML, {'module_x': x, 'module_y': y, 'wheel_radius': 0.0635,
                                       'max_wheel_speed': 20})
    assert '    module_x:     [0.3, 0.3, -0.3, -0.3]' in text and '# 前' in text
    assert '    wheel_radius: 0.0635' in text and '# 半径' in text
    assert '    max_wheel_speed: 20.0\n' in text and '    steer_speed: 6.0' in text
    back = sf.read_yaml_values(text)
    assert back['module_y'] == [0.2, -0.2, 0.2, -0.2] and back['max_wheel_speed'] == 20.0


def test_helpers():
    assert sf.c_float(2) == '2.0f' and sf.c_float(0.25) == '0.25f'
    assert sf.max_wheel_speed(25.0, -0.5) == 12.5


@pytest.mark.skipif(not os.path.exists(APP_CONFIG), reason='firmware/ が見つからない')
def test_real_files_can_be_read():
    # リポジトリの本物のファイルが、この形式で読めること(形式を変えたら気づけるように)
    with open(APP_CONFIG, encoding='utf-8') as f:
        assert set(sf.read_defines(f.read())) == set(sf.FIRMWARE_KEYS)
    with open(SWERVE_YAML, encoding='utf-8') as f:
        values = sf.read_yaml_values(f.read())
    assert len(values['module_x']) == len(values['module_names']) == len(values['device_ids'])
    assert sf.check_modules(values['module_names'], values['device_ids'], values['module_x'],
                            values['module_y']) == []


def test_yaml_names_and_ids_roundtrip():
    values = sf.read_yaml_values(YAML)
    assert values['module_names'] == ['front_left', 'front_right', 'rear_left', 'rear_right']
    assert values['device_ids'] == [1, 2, 3, 4]
    names, ids, x, y = sf.module_layout(3, 0.4, 0.3)
    text = sf.write_yaml_values(YAML, {'module_names': names, 'device_ids': ids,
                                       'module_x': x, 'module_y': y})
    assert '    module_names: [front, rear_left, rear_right]\n' in text
    assert '    device_ids:   [1, 2, 3]' in text and '# ID' in text
    back = sf.read_yaml_values(text)
    assert back['module_names'] == names and back['device_ids'] == [1, 2, 3]
    assert back['module_x'] == [0.2, -0.2, -0.2] and back['module_y'] == [0.0, 0.15, -0.15]


@pytest.mark.parametrize('count', sorted(sf.LAYOUTS))
def test_every_layout_is_valid(count):
    names, ids, x, y = sf.module_layout(count, 0.5, 0.4)
    assert len(names) == len(ids) == len(x) == len(y) == count
    assert sf.check_modules(names, ids, x, y) == []
    assert count in sf.LAYOUT_NOTES


def test_layout_rejects_unknown_count():
    with pytest.raises(ValueError):
        sf.module_layout(5, 0.5, 0.4)


def test_check_modules_finds_problems():
    ok = (['a', 'b'], [1, 2], [0.1, -0.1], [0.0, 0.0])
    assert sf.check_modules(*ok) == []
    assert sf.check_modules(['a'], [1], [0.0], [0.0])                       # 1 個
    assert sf.check_modules(['a', 'b'], [1], [0.1, -0.1], [0.0, 0.0])       # 数が違う
    assert sf.check_modules(['a', 'a'], [1, 2], [0.1, -0.1], [0.0, 0.0])    # 名前が重なる
    assert sf.check_modules(['a', 'b'], [3, 3], [0.1, -0.1], [0.0, 0.0])    # ID が重なる
    assert sf.check_modules(['a', 'b'], [0, 256], [0.1, -0.1], [0.0, 0.0])  # ID の範囲
    assert sf.check_modules(['左', 'b c'], [1, 2], [0.1, -0.1], [0.0, 0.0])  # 名前の文字
    assert sf.check_modules(['a', 'b'], [1, 2], [0.1, 0.1], [0.0, 0.0])     # 同じ位置
