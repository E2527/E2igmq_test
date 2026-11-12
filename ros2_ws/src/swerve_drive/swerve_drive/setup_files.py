"""
ギア比と寸法の設定ファイルを読み書きする(drive_setup の画面で使う).

画面にも ROS にも依存しないので、PC だけでテストできる(test/test_setup_files.py)。
書き換えるのは値だけで、行の後ろのコメントやほかの行はそのまま残す。

    firmware/motor_node_f303k8/Core/Inc/app_config.h … STM32 の設定(#define 名前 値)
    ros2_ws/src/swerve_drive/config/swerve.yaml       … Pi の設定(名前: 値)
"""

import re

# 画面で扱う STM32 の設定
FIRMWARE_KEYS = ('APP_COUNTS_PER_REV', 'APP_SWERVE_STEER_RATIO', 'APP_SWERVE_WHEEL_RATIO',
                 'APP_SWERVE_COUPLING', 'APP_SWERVE_MAX_MOTOR_SPEED')
# 画面で扱う Pi の設定
YAML_KEYS = ('module_x', 'module_y', 'wheel_radius', 'max_wheel_speed')

_DEFINE = r'^(#define[ \t]+{name}[ \t]+)([-+0-9.eE]+)f?'


def read_defines(text, names=FIRMWARE_KEYS):
    """C のヘッダから「#define 名前 数」を読み、{名前: 数} を返す."""
    values = {}
    for name in names:
        match = re.search(_DEFINE.format(name=name), text, re.MULTILINE)
        if match is None:
            raise KeyError(f'{name} が見つかりません')
        values[name] = float(match.group(2))
    return values


def write_defines(text, values):
    """「#define 名前 数」の数だけを書き換えた文字列を返す(後ろのコメントは残す)."""
    for name, value in values.items():
        text, count = re.subn(_DEFINE.format(name=name),
                              lambda m, v=value: m.group(1) + c_float(v), text, count=1,
                              flags=re.MULTILINE)
        if count == 0:
            raise KeyError(f'{name} が見つかりません')
    return text


def c_float(value):
    """C の float の書き方(例: 1440.0f、0.25f)."""
    text = repr(float(value))
    if 'e' not in text and '.' not in text:
        text += '.0'
    return text + 'f'


_YAML = r'^([ \t]+{name}:[ \t]*)([^#\n]*?)([ \t]*(#.*)?)$'


def read_yaml_values(text, names=YAML_KEYS):
    """YAML の「名前: 値」を読み、{名前: 数 か 数のリスト} を返す."""
    values = {}
    for name in names:
        match = re.search(_YAML.format(name=name), text, re.MULTILINE)
        if match is None:
            raise KeyError(f'{name} が見つかりません')
        raw = match.group(2).strip()
        if raw.startswith('['):
            values[name] = [float(v) for v in raw.strip('[]').split(',') if v.strip()]
        else:
            values[name] = float(raw)
    return values


def write_yaml_values(text, values):
    """YAML の「名前: 値」の値だけを書き換えた文字列を返す(後ろのコメントは残す)."""
    for name, value in values.items():
        if isinstance(value, (list, tuple)):
            new = '[' + ', '.join(_number(v) for v in value) + ']'
        else:
            new = _number(value)

        def replace(m, new=new):
            head, old, tail, comment = m.group(1), m.group(2), m.group(3), m.group(4)
            if not comment:
                return head + new
            # 値の長さが変わっても、コメントの位置がなるべく同じ列にそろうように空白を足す
            spaces = len(tail) - len(tail.lstrip())
            return head + new + ' ' * max(1, len(old) + spaces - len(new)) + comment

        text, count = re.subn(_YAML.format(name=name), replace, text, count=1,
                              flags=re.MULTILINE)
        if count == 0:
            raise KeyError(f'{name} が見つかりません')
    return text


def _number(value):
    value = float(value)
    text = f'{value:.6g}'
    return text if ('.' in text or 'e' in text) else text + '.0'


def module_layout(length, width):
    """前後の間隔 length [m] と左右の間隔 width [m] から、4 モジュールの (x の並び, y の並び)."""
    hx, hy = length / 2.0, width / 2.0
    return [hx, hx, -hx, -hx], [hy, -hy, hy, -hy]


def max_wheel_speed(max_motor_speed, wheel_ratio):
    """STM32 が出せるタイヤの速さの上限 [rad/s](モーターの上限 × タイヤのギア比)."""
    return abs(max_motor_speed * wheel_ratio)
