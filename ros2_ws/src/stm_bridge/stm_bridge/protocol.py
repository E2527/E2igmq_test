"""
Pi と STM32 の間のシリアルバス用パケット形式.

フレームの形(STM32 側の firmware/stm32_bus/bus_protocol.c と同じ):

    0xAA 0x55 | ID | CMD | LEN | PAYLOAD(LEN バイト) | CRC16(下位, 上位)

- ID      … 宛先の STM32 の番号(1〜247)。0 は予約
- CMD     … コマンド。子機(STM32)の返信は CMD に 0x80 を足した値
- LEN     … PAYLOAD のバイト数(0〜MAX_PAYLOAD)
- CRC16   … ID から PAYLOAD の最後までの CRC-16/MODBUS(リトルエンディアン)

CMD_DATA の PAYLOAD は float32(リトルエンディアン)の並び。
"""

from dataclasses import dataclass
import struct

SYNC = b'\xaa\x55'
HEADER_SIZE = 5           # SYNC(2) + ID + CMD + LEN
CRC_SIZE = 2
MAX_PAYLOAD = 64          # float32 なら 16 個まで
MAX_FRAME_SIZE = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE

CMD_DATA = 0x01           # 指令値を送り、状態を返してもらう
CMD_PING = 0x02           # 生きているか確認する
REPLY_FLAG = 0x80


@dataclass
class Frame:
    dev_id: int
    cmd: int
    payload: bytes


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def encode(dev_id: int, cmd: int, payload: bytes = b'') -> bytes:
    if not 0 <= dev_id <= 0xFF or not 0 <= cmd <= 0xFF:
        raise ValueError('dev_id と cmd は 0〜255 で指定してください')
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f'payload は {MAX_PAYLOAD} バイトまでです({len(payload)} バイト)')
    body = bytes((dev_id, cmd, len(payload))) + bytes(payload)
    return SYNC + body + struct.pack('<H', crc16_modbus(body))


def pack_floats(values) -> bytes:
    return struct.pack(f'<{len(values)}f', *values)


def unpack_floats(payload: bytes) -> list:
    if len(payload) % 4 != 0:
        raise ValueError(f'float32 の並びとして読めない長さです({len(payload)} バイト)')
    return list(struct.unpack(f'<{len(payload) // 4}f', payload))


class FrameParser:
    """受信したバイト列からフレームを取り出す。ノイズや途中切れがあっても次の SYNC から読み直す."""

    def __init__(self):
        self._buf = bytearray()
        self.crc_errors = 0

    def reset(self):
        self._buf.clear()

    def feed(self, data: bytes) -> list:
        self._buf += data
        frames = []
        while True:
            start = self._buf.find(SYNC)
            if start < 0:
                # 最後の 1 バイトが SYNC の前半かもしれないので残す
                del self._buf[:-1]
                break
            del self._buf[:start]
            if len(self._buf) < HEADER_SIZE:
                break
            length = self._buf[4]
            if length > MAX_PAYLOAD:
                del self._buf[:1]      # 偽の SYNC だったので 1 バイト進めて探し直す
                continue
            total = HEADER_SIZE + length + CRC_SIZE
            if len(self._buf) < total:
                break
            body = bytes(self._buf[2:HEADER_SIZE + length])
            (crc,) = struct.unpack_from('<H', self._buf, HEADER_SIZE + length)
            if crc16_modbus(body) == crc:
                frames.append(Frame(body[0], body[1], body[3:]))
                del self._buf[:total]
            else:
                self.crc_errors += 1
                del self._buf[:1]
        return frames
