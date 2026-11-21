import math

from swerve_drive import sim_faults as sf


def test_defaults_mean_no_faults():
    f = sf.parse_faults('')
    assert f['dead'] == frozenset() and f['drop'] == 0.0 and f['lidar_noise'] is None
    assert sf.filter_state(f, 1, [0.5, 2.0]) == [0.5, 2.0]


def test_parse_and_clamp():
    f = sf.parse_faults('{"dead": [2, "3"], "nan": [1], "drop": 1.5, "slip": -1,'
                        ' "drift_deg_s": "x", "lidar_noise": 0.05}')
    assert f['dead'] == {2, 3} and f['nan'] == {1}
    assert f['drop'] == 1.0 and f['slip'] == 0.0 and f['drift_deg_s'] == 0.0
    assert f['lidar_noise'] == 0.05
    assert sf.parse_faults('[1, 2]') == sf.parse_faults('')      # 形がおかしければ故障なし
    assert sf.parse_faults('not json') == sf.parse_faults('')


def test_round_trip():
    f = sf.parse_faults('{"dead": [4, 1], "slip": 0.2}')
    assert sf.parse_faults(sf.dump_faults(f)) == f


def test_filter_state():
    f = sf.parse_faults('{"dead": [2], "nan": [3], "drop": 0.5}')
    assert sf.filter_state(f, 2, [1.0, 1.0], rand=lambda: 0.9) is None      # 止めている
    out = sf.filter_state(f, 3, [1.0, 1.0], rand=lambda: 0.9)
    assert all(math.isnan(v) for v in out)
    assert sf.filter_state(f, 1, [1.0, 2.0], rand=lambda: 0.9) == [1.0, 2.0]
    assert sf.filter_state(f, 1, [1.0, 2.0], rand=lambda: 0.1) is None      # 落とした
