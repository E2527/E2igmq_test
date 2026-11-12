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
    x, y = sf.module_layout(0.6, 0.4)
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
        assert len(sf.read_yaml_values(f.read())['module_x']) == 4
