from swerve_drive.cmd_mux import CmdMux


def make():
    return CmdMux([('teleop', 100, 0.5), ('nav', 10, 0.5)])


def test_nothing_received():
    assert make().active(0.0) is None


def test_nav_passes_when_alone():
    mux = make()
    assert mux.on_input('nav', 0.0)
    assert mux.active(0.1) == 'nav'


def test_teleop_wins_over_nav():
    mux = make()
    assert mux.on_input('nav', 0.0)
    assert mux.on_input('teleop', 0.1)
    assert not mux.on_input('nav', 0.2)     # 手で操縦している間は捨てる
    assert mux.active(0.3) == 'teleop'


def test_nav_resumes_after_teleop_timeout():
    mux = make()
    mux.on_input('teleop', 0.0)
    assert not mux.on_input('nav', 0.4)
    assert mux.on_input('nav', 0.6)         # 0.5 秒届かなかったので、自動操縦にもどる
    assert mux.active(0.6) == 'nav'


def test_everything_times_out():
    mux = make()
    mux.on_input('nav', 0.0)
    assert mux.active(1.0) is None
