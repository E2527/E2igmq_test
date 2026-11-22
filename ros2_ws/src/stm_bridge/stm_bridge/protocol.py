"""
Pi と STM32 の間の CAN メッセージの決まり.

STM32 側の firmware/stm32_can/can_node.c と同じ決まりにする。標準フォーマット(11 ビット ID)を使う。

    0x100 + ID … Pi → STM32 の指令値(ID は基板の番号 1〜255)
    0x200 + ID … STM32 → Pi の状態
    0x300 + ID … Pi → STM32 の設定値(ゲインなど)の読み書き(firmware/stm32_can/params.h)
    0x400 + ID … STM32 → Pi の、その返事

CAN は ID が小さいほど優先されるので、指令は状態より先に届く。
データは float32(リトルエンディアン)の並びで、1 メッセージ 8 バイト = 2 個まで。
データが 0 バイトの指令は「止まる」の意味。

STM32 1 枚でモーター 2 個を速度制御する(デューティ比は STM32 が PID で決める):
    指令 … [モーター 1 の目標速度, モーター 2 の目標速度] [rad/s]
    状態 … [モーター 1 の実際の速度, モーター 2 の実際の速度] [rad/s]
"""

import math
import struct

COMMAND_BASE = 0x100
STATE_BASE = 0x200
PARAM_REQUEST_BASE = 0x300
PARAM_REPLY_BASE = 0x400
MAX_DEVICE_ID = 0xFF
MAX_DATA = 8
MAX_FLOATS = MAX_DATA // 4

# 受信フィルタ用: (ID, マスク) が一致する ID だけを受け取る。0x100 の位で、向きを分ける
STATE_FILTER = (0x000, 0x100)       # STM32 → Pi(0x200 の状態、0x400 の返事)
COMMAND_FILTER = (0x100, 0x100)     # Pi → STM32(0x100 の指令、0x300 の頼みごと)

# ---- 設定値(ゲインなど)の読み書き。番号は firmware/stm32_can/params.h と同じ ----
#   頼みごと: [番号](1 バイト)= 読む / [番号, 0, 0, 0, float の値](8 バイト)= 書く
#   返事:     [番号, 結果, 0, 0, float の今の値](8 バイト)

# 番号 → (名前, 説明, 単位, どのファームウェアにあるか('motors' / 'swerve' / 'all'))
PARAMS = {
    0: ('firmware', 'ファームウェアの種類(1 = モーター 2 個、2 = スワーブ)', '', 'all'),
    1: ('speed_kp', '速度の P ゲイン', 'デューティ比 / (rad/s)', 'all'),
    2: ('speed_ki', '速度の I ゲイン', 'デューティ比 / rad', 'all'),
    3: ('speed_kff', '速度のフィードフォワード(1 / 最高速度)', 'デューティ比 / (rad/s)', 'all'),
    4: ('max_duty', 'デューティ比の上限', '', 'all'),
    10: ('angle_kp', '向きの P ゲイン', '(rad/s) / rad', 'swerve'),
    11: ('angle_kd', '向きの D ゲイン(ブレーキ)', '', 'swerve'),
    12: ('angle_ff', '向きのフィードフォワード(0〜1)', '', 'swerve'),
    13: ('max_steer_rate', '向きを変える最大の速さ', 'rad/s', 'swerve'),
    20: ('angle', '今の向き(書くと、今の向きをその値にする)', 'rad', 'swerve'),
}
PARAM_IDS = {name: pid for pid, (name, _, _, _) in PARAMS.items()}
FIRMWARE_KINDS = {1: 'motors', 2: 'swerve'}

PARAM_OK, PARAM_UNKNOWN, PARAM_OUT_OF_RANGE, PARAM_READ_ONLY, PARAM_BAD_REQUEST = range(5)
PARAM_STATUS_TEXT = {
    PARAM_OK: 'OK', PARAM_UNKNOWN: 'この基板にはない', PARAM_OUT_OF_RANGE: '範囲の外(書かなかった)',
    PARAM_READ_ONLY: '読むだけ', PARAM_BAD_REQUEST: '頼み方がおかしい',
}


def _check_device_id(dev_id):
    if not 1 <= dev_id <= MAX_DEVICE_ID:
        raise ValueError(f'基板の ID は 1〜{MAX_DEVICE_ID} で指定してください({dev_id})')


def command_id(dev_id: int) -> int:
    _check_device_id(dev_id)
    return COMMAND_BASE + dev_id


def state_id(dev_id: int) -> int:
    _check_device_id(dev_id)
    return STATE_BASE + dev_id


def param_request_id(dev_id: int) -> int:
    _check_device_id(dev_id)
    return PARAM_REQUEST_BASE + dev_id


def param_reply_id(dev_id: int) -> int:
    _check_device_id(dev_id)
    return PARAM_REPLY_BASE + dev_id


def pack_param_request(param_id: int, value=None) -> bytes:
    """設定値の頼みごと。value が None なら読む、数なら書く."""
    if not 0 <= int(param_id) <= 0xFF:
        raise ValueError(f'設定値の番号は 0〜255 です({param_id})')
    if value is None:
        return bytes([int(param_id)])
    if not math.isfinite(value):
        raise ValueError(f'設定値に NaN や inf は送れません({value})')
    return struct.pack('<B3xf', int(param_id), float(value))


def unpack_param_request(data: bytes):
    """頼みごとを (番号, 値) にする(読むときは値が None)。読めなければ ValueError."""
    if len(data) == 1:
        return data[0], None
    if len(data) == 8:
        param_id, value = struct.unpack('<B3xf', data)
        return param_id, value
    raise ValueError(f'設定値の頼みごとの長さがおかしい({len(data)} バイト)')


def pack_param_reply(param_id: int, status: int, value: float) -> bytes:
    return struct.pack('<BB2xf', param_id, status, value)


def unpack_param_reply(data: bytes):
    """返事を (番号, 結果, 今の値) にする。読めなければ ValueError."""
    if len(data) != 8:
        raise ValueError(f'設定値の返事の長さがおかしい({len(data)} バイト)')
    return struct.unpack('<BB2xf', data)


def request_from_list(values) -> bytes:
    """stm<ID>/param の中身([番号] か [番号, 値])を、頼みごとのバイト列にする。おかしければ ValueError."""
    if len(values) not in (1, 2) or not math.isfinite(values[0]):
        raise ValueError(f'stm<ID>/param は [番号] か [番号, 値] です({list(values)})')
    return pack_param_request(int(round(values[0])), values[1] if len(values) == 2 else None)


def reply_to_list(data: bytes) -> list:
    """返事のバイト列を、stm<ID>/param_reply の中身 [番号, 結果, 今の値] にする."""
    param_id, status, value = unpack_param_reply(data)
    return [float(param_id), float(status), float(value)]


def device_of(can_id: int, base: int):
    """CAN の ID から基板の ID を求める。base の範囲でなければ None."""
    dev_id = can_id - base
    return dev_id if 1 <= dev_id <= MAX_DEVICE_ID else None


def pack_floats(values) -> bytes:
    if len(values) > MAX_FLOATS:
        raise ValueError(f'CAN の 1 メッセージに入る float は {MAX_FLOATS} 個までです({len(values)} 個)')
    return struct.pack(f'<{len(values)}f', *values)


def unpack_floats(data: bytes) -> list:
    if len(data) % 4 != 0:
        raise ValueError(f'float32 の並びとして読めない長さです({len(data)} バイト)')
    return list(struct.unpack(f'<{len(data) // 4}f', data))
