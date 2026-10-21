"""
CAN の送受信.

    SocketCanBus … Linux の SocketCAN(can0 など)。本番用
    SerialBus    … USB シリアル(Nucleo の ST-LINK の仮想 COM ポートなど)で CAN と同じ中身を送る
    SimBus       … UDP で CAN を真似る。カーネルに CAN がない環境(WSL など)で fake_stm と試す用

open_bus('can0') なら SocketCanBus、open_bus('serial:/dev/ttyACM0') なら SerialBus
(通信速度を変えるときは 'serial:/dev/ttyACM0@460800')、open_bus('sim') なら SimBus を返す。

シリアルでのフレーム(STM32 側の firmware/stm32_can/uart_node.c と同じ):

    0xAA 0x55 | ID(下位, 上位) | LEN | DATA(LEN バイト、8 まで) | CRC16(下位, 上位)

ID は CAN の標準 ID(0x101 など)、CRC16 は ID から DATA の最後までの CRC-16/MODBUS。
"""

import errno
import os
import select
import socket
import struct
import termios
import tty

# Linux の struct can_frame: can_id(4) + dlc(1) + 予約(3) + data(8)
CAN_FRAME = struct.Struct('=IB3x8s')
CAN_EFF_FLAG = 0x80000000   # 拡張フォーマット(29 ビット ID)
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x7FF

SIM_PORTS = {'bridge': 47000, 'device': 47001}

# 送信待ちがあふれた(バスに他の基板がいない、配線が切れているなど)。開き直しても直らないので続ける
TRANSIENT_ERRORS = (errno.ENOBUFS, errno.EAGAIN)


def pack_frame(can_id: int, data: bytes) -> bytes:
    if len(data) > 8:
        raise ValueError('CAN のデータは 8 バイトまでです')
    return CAN_FRAME.pack(can_id & CAN_SFF_MASK, len(data), bytes(data).ljust(8, b'\x00'))


def unpack_frame(raw: bytes):
    """標準フォーマットのデータフレームなら (ID, data) を、それ以外は None を返す."""
    can_id, dlc, data = CAN_FRAME.unpack(raw[:CAN_FRAME.size])
    if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG):
        return None
    return can_id & CAN_SFF_MASK, data[:min(dlc, 8)]


class SocketCanBus:

    def __init__(self, interface: str, rx_filter):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_id, mask = rx_filter
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                             struct.pack('=II', can_id, mask | CAN_EFF_FLAG | CAN_RTR_FLAG))
        self.sock.bind((interface,))

    def send(self, can_id: int, data: bytes):
        self.sock.send(pack_frame(can_id, data))

    def recv(self, timeout: float):
        self.sock.settimeout(timeout)
        try:
            raw = self.sock.recv(CAN_FRAME.size)
        except (socket.timeout, BlockingIOError):   # timeout=0 のときは BlockingIOError になる
            return None
        return unpack_frame(raw)

    def close(self):
        self.sock.close()


class SimBus:
    """UDP の 127.0.0.1 で、ブリッジと fake_stm の 2 者だけをつなぐ."""

    def __init__(self, role: str, rx_filter):
        self.rx_filter = rx_filter
        self.peer = ('127.0.0.1', SIM_PORTS['device' if role == 'bridge' else 'bridge'])
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('127.0.0.1', SIM_PORTS[role]))

    def send(self, can_id: int, data: bytes):
        self.sock.sendto(pack_frame(can_id, data), self.peer)

    def recv(self, timeout: float):
        self.sock.settimeout(timeout)
        try:
            raw = self.sock.recv(CAN_FRAME.size)
        except (socket.timeout, BlockingIOError):   # timeout=0 のときは BlockingIOError になる
            return None
        frame = unpack_frame(raw)
        can_id, mask = self.rx_filter
        if frame is None or (frame[0] & mask) != (can_id & mask):
            return None
        return frame

    def close(self):
        self.sock.close()


SERIAL_SYNC = b'\xaa\x55'
SERIAL_DEFAULT_BAUD = 115200


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def encode_serial_frame(can_id: int, data: bytes) -> bytes:
    if len(data) > 8:
        raise ValueError('CAN のデータは 8 バイトまでです')
    body = struct.pack('<HB', can_id & CAN_SFF_MASK, len(data)) + bytes(data)
    return SERIAL_SYNC + body + struct.pack('<H', crc16_modbus(body))


class SerialFrameParser:
    """シリアルで届いたバイト列から (ID, data) を取り出す。ノイズや途中切れは読み飛ばす."""

    HEADER = 5      # SYNC(2) + ID(2) + LEN

    def __init__(self):
        self._buf = bytearray()
        self.crc_errors = 0

    def feed(self, chunk: bytes) -> list:
        self._buf += chunk
        frames = []
        while True:
            start = self._buf.find(SERIAL_SYNC)
            if start < 0:
                del self._buf[:-1]      # 最後の 1 バイトは SYNC の前半かもしれないので残す
                return frames
            del self._buf[:start]
            if len(self._buf) < self.HEADER:
                return frames
            length = self._buf[4]
            if length > 8:
                del self._buf[:1]       # 偽の SYNC。1 バイト進めて探し直す
                continue
            total = self.HEADER + length + 2
            if len(self._buf) < total:
                return frames
            body = bytes(self._buf[2:self.HEADER + length])
            (crc,) = struct.unpack_from('<H', self._buf, self.HEADER + length)
            if crc == crc16_modbus(body):
                (can_id,) = struct.unpack_from('<H', body)
                frames.append((can_id & CAN_SFF_MASK, body[3:]))
                del self._buf[:total]
            else:
                self.crc_errors += 1
                del self._buf[:1]


def _baud_constant(baud: int):
    name = f'B{baud}'
    if not hasattr(termios, name):
        raise ValueError(f'この通信速度は使えません: {baud}')
    return getattr(termios, name)


class SerialBus:
    """USB シリアルで CAN と同じ (ID, data) をやり取りする(1 対 1)."""

    def __init__(self, path: str, rx_filter, baud: int = SERIAL_DEFAULT_BAUD):
        self.rx_filter = rx_filter
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            tty.setraw(self.fd)
            attrs = termios.tcgetattr(self.fd)
            attrs[4] = attrs[5] = _baud_constant(baud)      # 入力・出力の速度
            attrs[2] |= termios.CLOCAL | termios.CREAD
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except termios.error as e:
            os.close(self.fd)
            raise OSError(errno.EIO, f'シリアルポートの設定に失敗しました: {e}') from e
        except Exception:
            os.close(self.fd)
            raise
        self.parser = SerialFrameParser()
        self.pending = []

    def send(self, can_id: int, data: bytes):
        os.write(self.fd, encode_serial_frame(can_id, data))

    def recv(self, timeout: float):
        while True:
            while self.pending:
                frame = self.pending.pop(0)
                can_id, mask = self.rx_filter
                if (frame[0] & mask) == (can_id & mask):
                    return frame
            ready, _, _ = select.select([self.fd], [], [], max(0.0, timeout))
            if not ready:
                return None
            chunk = os.read(self.fd, 256)
            if not chunk:
                raise OSError(errno.EIO, 'シリアルポートが閉じられました(USB が抜けた?)')
            self.pending.extend(self.parser.feed(chunk))
            timeout = 0.0

    def close(self):
        os.close(self.fd)


def open_bus(interface: str, rx_filter, role: str = 'bridge'):
    if interface == 'sim':
        return SimBus(role, rx_filter)
    if interface.startswith('serial:'):
        path, _, baud = interface[len('serial:'):].partition('@')
        return SerialBus(path, rx_filter, int(baud) if baud else SERIAL_DEFAULT_BAUD)
    return SocketCanBus(interface, rx_filter)
