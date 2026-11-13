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
YAML_KEYS = ('module_names', 'device_ids', 'module_x', 'module_y', 'wheel_radius',
             'max_wheel_speed')

# 整数で書く Pi の設定(ほかの数は、ROS のパラメータの型が変わらないように、必ず小数で書く)
INT_KEYS = ('device_ids',)

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
            values[name] = [_parse_item(v) for v in raw.strip('[]').split(',') if v.strip()]
        else:
            values[name] = _parse_item(raw)
    return values


def _parse_item(raw):
    """YAML の値 1 個: 整数なら int、小数なら float、それ以外は文字列(名前)."""
    raw = raw.strip().strip('\'"')
    for kind in (int, float):
        try:
            return kind(raw)
        except ValueError:
            pass
    return raw


def write_yaml_values(text, values):
    """YAML の「名前: 値」の値だけを書き換えた文字列を返す(後ろのコメントは残す)."""
    for name, value in values.items():
        as_int = name in INT_KEYS
        if isinstance(value, (list, tuple)):
            new = '[' + ', '.join(_item(v, as_int) for v in value) + ']'
        else:
            new = _item(value, as_int)

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


def _item(value, as_int=False):
    """YAML に書く値 1 個(名前はそのまま、as_int なら整数、それ以外は小数点を付ける)."""
    if isinstance(value, str):
        return value
    if as_int:
        return str(int(value))
    return _number(value)


def _number(value):
    value = float(value)
    text = f'{value:.6g}'
    return text if ('.' in text or 'e' in text) else text + '.0'


# よく使うモジュールの並べ方。モジュールの数 → [(名前, 前後の位置, 左右の位置)]
# 位置は -1〜1 で、1 が「前後(左右)の間隔の半分」(前と左がプラス)
LAYOUTS = {
    2: [('front', 1, 0), ('rear', -1, 0)],
    3: [('front', 1, 0), ('rear_left', -1, 1), ('rear_right', -1, -1)],
    4: [('front_left', 1, 1), ('front_right', 1, -1), ('rear_left', -1, 1),
        ('rear_right', -1, -1)],
    6: [('front_left', 1, 1), ('front_right', 1, -1), ('middle_left', 0, 1),
        ('middle_right', 0, -1), ('rear_left', -1, 1), ('rear_right', -1, -1)],
}
LAYOUT_NOTES = {
    2: '前と後ろのまん中に 1 個ずつ(左右の間隔は使わない)',
    3: '前のまん中に 1 個、後ろの左右に 1 個ずつ(三角形)',
    4: '四隅に 1 個ずつ(長方形)',
    6: '左右に 3 個ずつ(前・まん中・後ろ)',
}


def module_layout(count, length, width):
    """
    モジュールの数と、前後の間隔 length [m]・左右の間隔 width [m] から、並べ方を作る.

    (名前の並び, STM32 の ID の並び, x の並び, y の並び) を返す。ID は 1 から順番。
    """
    if count not in LAYOUTS:
        raise ValueError(f'{count} 個の並べ方はありません({sorted(LAYOUTS)} から選ぶ)')
    hx, hy = length / 2.0, width / 2.0
    rows = LAYOUTS[count]
    return ([name for name, _, _ in rows], list(range(1, count + 1)),
            [fx * hx + 0.0 for _, fx, _ in rows], [fy * hy + 0.0 for _, _, fy in rows])


_NAME = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')


def check_modules(names, ids, xs, ys):
    """
    モジュールの設定がおかしくないか調べて、問題の説明の並びを返す(なければ空).

    swerve_node が起動できない設定(数がそろわない、ID の重複など)を、保存する前に見つける。
    """
    problems = []
    if not len(names) == len(ids) == len(xs) == len(ys):
        problems.append('名前・ID・位置の数がそろっていません')
        return problems
    if len(names) < 2:
        problems.append('モジュールは 2 個以上にしてください(1 個だと回転が求められません)')
    bad = [n for n in names if not _NAME.match(n)]
    if bad:
        problems.append(f'名前は半角の英数字と _ だけにしてください: {", ".join(bad)}')
    for label, values in (('名前が', names), ('ID が', ids)):
        twice = sorted({str(v) for v in values if values.count(v) > 1})
        if twice:
            problems.append(f'{label}重なっています: {", ".join(twice)}')
    out = [str(i) for i in ids if not 1 <= i <= 255]
    if out:
        problems.append(f'ID は 1〜255 にしてください: {", ".join(out)}')
    places = {(round(x, 4), round(y, 4)) for x, y in zip(xs, ys)}
    if len(names) >= 2 and len(places) == 1:
        problems.append('全部のモジュールが同じ位置にあります')
    return problems


def max_wheel_speed(max_motor_speed, wheel_ratio):
    """STM32 が出せるタイヤの速さの上限 [rad/s](モーターの上限 × タイヤのギア比)."""
    return abs(max_motor_speed * wheel_ratio)
