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
