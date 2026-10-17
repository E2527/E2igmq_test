"""
Pi と STM32 の間の CAN メッセージの決まり.

STM32 側の firmware/stm32_can/can_node.c と同じ決まりにする。標準フォーマット(11 ビット ID)を使う。

    0x100 + ID … Pi → STM32 の指令値(ID は基板の番号 1〜255)
    0x200 + ID … STM32 → Pi の状態

CAN は ID が小さいほど優先されるので、指令は状態より先に届く。
データは float32(リトルエンディアン)の並びで、1 メッセージ 8 バイト = 2 個まで。
データが 0 バイトの指令は「止まる」の意味。
"""

import struct

COMMAND_BASE = 0x100
STATE_BASE = 0x200
MAX_DEVICE_ID = 0xFF
MAX_DATA = 8
MAX_FLOATS = MAX_DATA // 4

# 受信フィルタ用: (ID, マスク) が一致する ID だけを受け取る
STATE_FILTER = (STATE_BASE, 0x700)
COMMAND_FILTER = (COMMAND_BASE, 0x700)


def _check_device_id(dev_id):
    if not 1 <= dev_id <= MAX_DEVICE_ID:
        raise ValueError(f'基板の ID は 1〜{MAX_DEVICE_ID} で指定してください({dev_id})')


def command_id(dev_id: int) -> int:
    _check_device_id(dev_id)
    return COMMAND_BASE + dev_id


def state_id(dev_id: int) -> int:
    _check_device_id(dev_id)
    return STATE_BASE + dev_id


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
