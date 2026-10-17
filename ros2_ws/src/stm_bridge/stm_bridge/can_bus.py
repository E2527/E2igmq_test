"""
CAN の送受信.

    SocketCanBus … Linux の SocketCAN(can0 など)。本番用
    SimBus       … UDP で CAN を真似る。カーネルに CAN がない環境(WSL など)で fake_stm と試す用

open_bus('can0') なら SocketCanBus、open_bus('sim') なら SimBus を返す。
"""

import errno
import socket
import struct

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


def open_bus(interface: str, rx_filter, role: str = 'bridge'):
    if interface == 'sim':
        return SimBus(role, rx_filter)
    return SocketCanBus(interface, rx_filter)
