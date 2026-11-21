import math

import pytest
from swerve_drive import console_state as cs
from swerve_drive.console_state import Arrow, ConsoleState, ERROR, OK, STALE, WARN


def test_command_and_actual_time_out():
    s = ConsoleState()
    snap = s.snapshot(0.0)
    assert snap.command == (0.0, 0.0, 0.0) and snap.actual is None

    s.on_cmd(0.5, 0.0, 1.0, now=1.0)
    s.on_odom(1.0, 2.0, 0.3, 0.49, 0.0, 0.98, now=1.0)
    snap = s.snapshot(1.1)
    assert snap.command == (0.5, 0.0, 1.0) and snap.actual == (0.49, 0.0, 0.98)
    assert snap.pose == (1.0, 2.0, 0.3) and not snap.capped

    snap = s.snapshot(1.7)          # 指令は 0.5 秒で途切れたとみなす(足回りも止まる)
    assert snap.command == (0.0, 0.0, 0.0) and snap.actual is not None
    assert s.snapshot(3.0).actual is None       # /odom も届かなくなった


def test_capped_when_actual_is_much_slower_than_command():
    s = ConsoleState()
    s.on_cmd(1.0, 0.0, 0.0, now=0.0)
    s.on_odom(0.0, 0.0, 0.0, 0.6, 0.0, 0.0, now=0.0)
    snap = s.snapshot(0.1)
    assert snap.capped and snap.level == WARN       # 状態の知らせがなくても、頭打ちは分かる
    s.on_odom(0.0, 0.0, 0.0, 0.95, 0.0, 0.0, now=0.1)
    assert not s.snapshot(0.2).capped


def test_trail_adds_points_only_when_moving():
    s = ConsoleState()
    for i in range(100):
        s.on_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, now=i * 0.02)      # 止まっている
    assert len(s.snapshot(2.0).trail) == 1
    for i in range(100):
        s.on_odom(i * 0.01, 0.0, 0.0, 0.5, 0.0, 0.0, now=2.0 + i * 0.02)
    trail = s.snapshot(4.0).trail
    assert 40 <= len(trail) <= 60 and trail[-1][0] > 0.9
    s.clear_trail()
    assert s.snapshot(4.0).trail == []


def test_arrows_and_drive_name():
    s = ConsoleState()
    s.on_arrows('omni', [Arrow(0.2, 0.2, 0.0, 0.1)], [Arrow(0.2, 0.2, 0.1, 0.08)], now=0.0)
    snap = s.snapshot(0.1)
    assert snap.drive == 'omni' and len(snap.targets) == 1 and len(snap.actuals) == 1
    assert s.snapshot(5.0).targets == []        # 届かなくなったら消す


def test_rows_levels_and_summary():
    s = ConsoleState()
    assert s.snapshot(0.0).level == STALE
    s.on_diagnostic('stm_bridge: stm1', OK, '通信しています', {'受信': '100 回/秒'}, now=0.0)
    s.on_diagnostic('swerve_drive: 指令', OK, '指令なし', {'stopped': 'false'}, now=0.0)
    snap = s.snapshot(0.5)
    assert snap.level == OK and snap.summary == 'すべて OK'
    assert [r.name for r in snap.rows] == ['stm_bridge: stm1', 'swerve_drive: 指令']
    assert snap.rows[0].values == '受信 100 回/秒'
    assert snap.rows[1].values == ''             # stopped は表示しない

    s.on_diagnostic('stm_bridge: stm2', ERROR, '状態が届いていません', {}, now=0.5)
    assert s.snapshot(0.6).level == ERROR

    snap = s.snapshot(10.0)                      # 全部が届かなくなった
    assert all(r.level == STALE for r in snap.rows) and snap.level == ERROR


def test_stop_follows_button_first_then_the_drive_report():
    s = ConsoleState()
    # ほかのコンソールで止めたあとに起動した: 足回りの知らせで「停止中」と分かる
    s.on_diagnostic('swerve_drive: 指令', WARN, '停止ボタンで止めています', {'stopped': 'true'}, 0.0)
    snap = s.snapshot(0.1)
    assert snap.stopped and snap.summary == '停止中'

    s.set_stop(False, now=1.0)                   # 再開を押した直後は、古い知らせに引きずられない
    assert not s.snapshot(1.1).stopped
    s.on_diagnostic('swerve_drive: 指令', OK, '指令なし', {'stopped': 'false'}, 1.5)
    assert not s.snapshot(3.0).stopped

    s.set_stop(True, now=4.0)
    assert s.snapshot(4.1).stopped
    assert s.snapshot(20.0).stopped              # 足回りの知らせが途切れても、自分の指示を覚えている


def test_map_scan_and_odom_in_map():
    state = cs.ConsoleState()
    snap = state.snapshot(0.0)
    assert snap.grid is None and snap.scan == [] and snap.map_pose == (0.0, 0.0, 0.0)
    state.on_map(4, 2, 0.5, -1.0, -0.5, [0, 100, -1, 0, 0, 0, 0, 100])
    state.on_scan([(1.0, 0.0), (0.0, 2.0)], now=1.0)
    state.on_odom(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, now=1.0)
    state.set_odom_in_map(0.5, 0.5, math.pi / 2)            # odom が地図の中で 90° 回っている
    snap = state.snapshot(1.5)
    assert snap.grid.width == 4 and snap.grid.cells[2] == 255 and snap.grid.version == 1
    assert snap.grid.bounds == (-1.0, 1.0, -0.5, 0.5)
    assert snap.scan == [(1.0, 0.0), (0.0, 2.0)]
    x, y, yaw = snap.map_pose
    assert (round(x, 6), round(y, 6), round(yaw, 6)) == (0.5, 1.5, round(math.pi / 2, 6))
    assert [tuple(round(v, 6) for v in p) for p in snap.map_trail()] == [(0.5, 1.5)]
    assert state.snapshot(3.0).scan == []                   # 古くなったら出さない
    state.on_map(1, 1, 1.0, 0.0, 0.0, b'\x00')
    assert state.snapshot(3.0).grid.version == 2


def test_compose():
    assert cs.compose((1.0, 2.0, 0.0), 0.5, 0.0) == (1.5, 2.0)
    x, y = cs.compose((0.0, 0.0, math.pi / 2), 1.0, 0.0)
    assert abs(x) < 1e-9 and abs(y - 1.0) < 1e-9


def test_scan_points_skip_bad_ranges_and_apply_the_mount():
    ranges = [1.0, float('inf'), 0.01, 2.0, float('nan')]
    points = cs.scan_points(ranges, angle_min=0.0, angle_increment=math.pi / 2, range_min=0.05,
                            range_max=10.0, laser_pose=(0.1, 0.0, 0.0))
    # 1 本目は前に 1 m、4 本目は 270°(右)に 2 m。LiDAR がロボットの中心より 0.1 m 前
    assert [tuple(round(v, 6) for v in p) for p in points] == [(1.1, 0.0), (0.1, -2.0)]


def test_stm_rows_rates_and_ages():
    state = cs.ConsoleState()
    for i in range(10):
        state.on_stm(2, 'command', [0.1 * i, 1.0], now=10.0 + i * 0.1)
    state.on_stm(2, 'state', [0.5, 0.9], now=10.8)
    state.on_stm(1, 'state', [0.0, 0.0], now=5.0)
    rows = state.snapshot(11.0).stm_rows
    assert [r.dev_id for r in rows] == [1, 2]
    one, two = rows
    assert one.command == () and one.command_age is None and one.state_rate == 0.0
    assert two.command[0] == pytest.approx(0.9) and two.command_rate == pytest.approx(10.0)
    assert two.state_age == pytest.approx(0.2)


def test_history_and_truth():
    state = cs.ConsoleState()
    state.on_cmd(0.5, 0.0, 0.0, now=0.0)
    state.on_stm(1, 'command', [1.0, 2.0], now=0.0)
    for i in range(30):
        state.sample(i * 1.0)
    snap = state.snapshot(29.0)
    assert len(snap.history) == 21 and snap.history[0].t == 9.0     # 20 秒分だけ残る
    assert state.snapshot(0.0).truth is None
    state.set_truth(1.0, 2.0, 0.5, now=30.0)
    assert state.snapshot(30.5).truth == (1.0, 2.0, 0.5)
    assert state.snapshot(32.0).truth is None
