import pytest
from stm_bridge import can_bus, protocol


def test_can_ids():
    assert protocol.command_id(1) == 0x101
    assert protocol.state_id(3) == 0x203
    assert protocol.device_of(0x203, protocol.STATE_BASE) == 3
    assert protocol.device_of(0x103, protocol.STATE_BASE) is None
    with pytest.raises(ValueError):
        protocol.command_id(0)


def test_commands_have_priority_over_states():
    # CAN は ID が小さいほうが優先される
    assert protocol.command_id(protocol.MAX_DEVICE_ID) < protocol.state_id(1)


def test_float_roundtrip_and_limit():
    data = protocol.pack_floats([1.5, -2.25])
    assert len(data) == 8
    assert protocol.unpack_floats(data) == [1.5, -2.25]
    assert protocol.pack_floats([]) == b''
    with pytest.raises(ValueError):
        protocol.pack_floats([1.0, 2.0, 3.0])


def test_frame_layout_matches_linux_can_frame():
    raw = can_bus.pack_frame(0x101, b'\x01\x02\x03')
    assert len(raw) == 16
    assert raw[:4] == (0x101).to_bytes(4, 'little')
    assert raw[4] == 3
    assert can_bus.unpack_frame(raw) == (0x101, b'\x01\x02\x03')


def test_unpack_ignores_extended_rtr_and_error_frames():
    for flag in (can_bus.CAN_EFF_FLAG, can_bus.CAN_RTR_FLAG, can_bus.CAN_ERR_FLAG):
        raw = can_bus.CAN_FRAME.pack(0x101 | flag, 0, bytes(8))
        assert can_bus.unpack_frame(raw) is None


def test_sim_bus_roundtrip_and_zero_timeout():
    bridge = can_bus.SimBus('bridge', protocol.STATE_FILTER)
    device = can_bus.SimBus('device', protocol.COMMAND_FILTER)
    try:
        assert bridge.recv(0.0) is None   # 何も届いていなければ、待たずに None
        bridge.send(protocol.command_id(1), protocol.pack_floats([2.0]))
        assert device.recv(1.0) == (0x101, protocol.pack_floats([2.0]))
        device.send(protocol.command_id(2), b'')   # フィルタに合わない ID は捨てる
        assert bridge.recv(0.1) is None
    finally:
        bridge.close()
        device.close()


def test_serial_frame_layout():
    frame = can_bus.encode_serial_frame(0x101, b'\x01\x02')
    assert frame[:2] == b'\xaa\x55'
    assert frame[2:5] == b'\x01\x01\x02'          # ID 下位・上位、LEN
    assert frame[5:7] == b'\x01\x02'
    assert len(frame) == 9
    assert can_bus.crc16_modbus(b'123456789') == 0x4B37


def test_serial_parser_handles_split_noise_and_crc_errors():
    good1 = can_bus.encode_serial_frame(0x201, protocol.pack_floats([1.0, 2.0]))
    good2 = can_bus.encode_serial_frame(0x202, b'')
    bad = bytearray(can_bus.encode_serial_frame(0x203, b'\x00'))
    bad[5] ^= 0xFF
    stream = b'\x00\xaa\x13' + good1 + bytes(bad) + b'\xff' + good2
    parser = can_bus.SerialFrameParser()
    frames = []
    for i in range(len(stream)):
        frames += parser.feed(stream[i:i + 1])     # 1 バイトずつ届いても読める
    assert frames == [(0x201, protocol.pack_floats([1.0, 2.0])), (0x202, b'')]
    assert parser.crc_errors == 1


def test_serial_bus_roundtrip_over_pty():
    import os
    master, slave = os.openpty()
    bus = can_bus.open_bus('serial:' + os.ttyname(slave), protocol.STATE_FILTER)
    try:
        bus.send(0x101, protocol.pack_floats([3.0]))
        assert can_bus.SerialFrameParser().feed(os.read(master, 64)) == [
            (0x101, protocol.pack_floats([3.0]))]
        os.write(master, can_bus.encode_serial_frame(0x105, b'') +           # フィルタに合わない
                 can_bus.encode_serial_frame(0x201, protocol.pack_floats([4.0])))
        assert bus.recv(1.0) == (0x201, protocol.pack_floats([4.0]))
        assert bus.recv(0.0) is None
    finally:
        bus.close()
        os.close(master)
        os.close(slave)
