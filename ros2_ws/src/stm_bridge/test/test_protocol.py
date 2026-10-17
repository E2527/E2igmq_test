import pytest
from stm_bridge import protocol


def test_crc16_modbus_check_value():
    # CRC-16/MODBUS の標準の確認値
    assert protocol.crc16_modbus(b'123456789') == 0x4B37


def test_encode_layout():
    frame = protocol.encode(3, protocol.CMD_DATA, b'\x01\x02')
    assert frame[:2] == protocol.SYNC
    assert frame[2:5] == bytes((3, protocol.CMD_DATA, 2))
    assert frame[5:7] == b'\x01\x02'
    assert len(frame) == protocol.HEADER_SIZE + 2 + protocol.CRC_SIZE


def test_encode_rejects_too_long_payload():
    with pytest.raises(ValueError):
        protocol.encode(1, protocol.CMD_DATA, bytes(protocol.MAX_PAYLOAD + 1))


def test_roundtrip_floats():
    values = [1.5, -2.25, 0.0]
    frame = protocol.encode(1, protocol.CMD_DATA, protocol.pack_floats(values))
    (parsed,) = protocol.FrameParser().feed(frame)
    assert (parsed.dev_id, parsed.cmd) == (1, protocol.CMD_DATA)
    assert protocol.unpack_floats(parsed.payload) == values


def test_parser_handles_split_and_noise():
    a = protocol.encode(1, protocol.CMD_DATA, b'abcd')
    b = protocol.encode(2, protocol.CMD_PING)
    stream = b'\x00\xaa\x13' + a + b'\xff' + b
    parser = protocol.FrameParser()
    frames = []
    for i in range(len(stream)):
        frames += parser.feed(stream[i:i + 1])  # 1 バイトずつ届いても読める
    assert [(f.dev_id, f.cmd, f.payload) for f in frames] == [
        (1, protocol.CMD_DATA, b'abcd'), (2, protocol.CMD_PING, b'')]


def test_parser_drops_corrupted_frame():
    bad = bytearray(protocol.encode(1, protocol.CMD_DATA, b'abcd'))
    bad[6] ^= 0xFF
    good = protocol.encode(2, protocol.CMD_DATA, b'')
    parser = protocol.FrameParser()
    frames = parser.feed(bytes(bad) + good)
    assert [f.dev_id for f in frames] == [2]
    assert parser.crc_errors == 1
