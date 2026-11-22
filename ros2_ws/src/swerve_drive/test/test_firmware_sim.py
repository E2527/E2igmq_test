import math
import os

import pytest
from swerve_drive import firmware_sim


def _firmware_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    for d in (os.environ.get('FIRMWARE_DIR', ''), firmware_sim.DEFAULT_FIRMWARE_DIR,
              os.path.join(here, '..', '..', '..', '..', 'firmware')):
        if d and os.path.exists(os.path.join(d, 'sim', 'firmware_sim.c')):
            return d
    return None


FIRMWARE_DIR = _firmware_dir()
pytestmark = pytest.mark.skipif(FIRMWARE_DIR is None, reason='firmware/ が見つからない')


@pytest.fixture(scope='module')
def lib():
    return firmware_sim.build(FIRMWARE_DIR)


def test_swerve_module_turns_and_drives(lib):
    sim = firmware_sim.SwerveSim(lib)
    try:
        out = sim.run(math.pi / 2, 3.0, True, 2.0)    # 90° を向いて 3 rad/s
        angle, wheel, true_angle, true_wheel = out
        assert abs(angle - math.pi / 2) < 0.05
        assert abs(wheel - 3.0) < 0.3
        assert abs(true_angle - angle) < 0.02           # ギア比が正しければ、測った値と本当の値が合う
        assert abs(true_wheel - wheel) < 0.3
        out = sim.run(math.pi / 2, 3.0, False, 1.0)    # 指令が途切れたら止まる
        assert abs(out[3]) < 0.1
    finally:
        sim.close()


def test_wrong_gear_ratio_shows_up(lib):
    # ファームウェアの設定より、本当の機構の向きのギア比が 2 倍だと、本当の向きは行き過ぎる
    sim = firmware_sim.SwerveSim(lib, mech_steer_ratio=0.5)
    try:
        angle, _, true_angle, _ = sim.run(0.5, 0.0, True, 2.0)
        assert abs(angle - 0.5) < 0.05 and abs(true_angle - 1.0) < 0.05
    finally:
        sim.close()


def test_board_follows_speed_targets(lib):
    sim = firmware_sim.BoardSim(lib)
    try:
        s1, s2 = sim.run(10.0, -5.0, True, 1.0)
        assert abs(s1 - 10.0) < 0.5 and abs(s2 + 5.0) < 0.5
        s1, s2 = sim.run(10.0, -5.0, False, 1.0)
        assert abs(s1) < 0.5 and abs(s2) < 0.5
    finally:
        sim.close()


def test_gains_can_be_changed_over_the_link(lib):
    """ゲインの読み書き(通信の 0x300 + ID と同じ頼みごと)で、ファームウェアの動きが変わる."""
    import struct
    sim = firmware_sim.SwerveSim(lib)
    try:
        def ask(param_id, value=None):
            req = bytes([param_id]) if value is None else struct.pack('<B3xf', param_id, value)
            return struct.unpack('<BB2xf', sim.param(req))

        assert ask(0) == (0, 0, 2.0)                    # ファームウェアの種類: スワーブ
        param_id, status, kp = ask(10)                  # 向きの P ゲイン(app_config.h の値)
        assert status == 0 and kp > 0
        assert ask(10, 2.0) == (10, 0, 2.0)
        assert ask(10, 1000.0)[1] == 2                  # 範囲の外は書かない
        assert ask(99)[1] == 1                          # ない番号

        # P ゲインを小さくすると、向きが合うのが遅くなる
        slow = sim.run(1.0, 0.0, True, 0.1)[0]
        ask(20, 0.0)                                    # 今の向きを 0 にする
        assert abs(ask(20)[2]) < 1e-6
        ask(10, kp)
        fast = sim.run(1.0, 0.0, True, 0.1)[0]
        assert fast > slow
    finally:
        sim.close()
