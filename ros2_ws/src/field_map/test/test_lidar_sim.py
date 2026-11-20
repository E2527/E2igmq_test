import math

from field_map import lidar_sim as ls
import pytest


def room(width=20, height=10, resolution=0.1):
    """まわりが壁の部屋(x 0〜2 m、y 0〜1 m)."""
    cells = []
    for r in range(height):
        for c in range(width):
            edge = r in (0, height - 1) or c in (0, width - 1)
            cells.append(100 if edge else 0)
    return ls.GridMap(width, height, resolution, 0.0, 0.0, cells)


def test_hits_the_walls_in_four_directions():
    grid = room()
    x, y = 1.0, 0.5
    # 右の壁はマス 19(x 1.9 m〜)、左の壁はマス 0(〜0.1 m)、上は 0.9 m〜、下は 〜0.1 m
    assert ls.cast(grid, x, y, 0.0, 10) == pytest.approx(0.9)
    assert ls.cast(grid, x, y, math.pi, 10) == pytest.approx(0.9)
    assert ls.cast(grid, x, y, math.pi / 2, 10) == pytest.approx(0.4)
    assert ls.cast(grid, x, y, -math.pi / 2, 10) == pytest.approx(0.4)


def test_diagonal_and_range_limit():
    grid = room()
    d = ls.cast(grid, 1.0, 0.5, math.pi / 4, 10)
    assert d == pytest.approx(0.4 * math.sqrt(2), abs=0.01)
    assert ls.cast(grid, 1.0, 0.5, 0.0, 0.5) == math.inf        # 0.9 m 先の壁は届かない


def test_outside_the_map_is_nothing():
    cells = [0] * 100
    grid = ls.GridMap(10, 10, 0.1, 0.0, 0.0, cells)
    assert ls.cast(grid, 0.5, 0.5, 0.3, 20) == math.inf
    assert ls.cast(grid, -1.0, 0.5, math.pi, 20) == math.inf    # 外から、外向き


def test_ray_from_outside_can_enter_the_map():
    cells = [0] * 100
    cells[5 * 10 + 5] = 100                                     # まん中に 1 マス
    grid = ls.GridMap(10, 10, 0.1, 0.0, 0.0, cells)
    assert ls.cast(grid, -1.0, 0.55, 0.0, 20) == pytest.approx(1.5)


def test_inside_a_wall_is_zero():
    assert ls.cast(room(), 0.05, 0.5, 0.0, 10) == 0.0


def test_scan_follows_the_robot_heading():
    grid = room()
    ranges = ls.scan(grid, 1.0, 0.5, math.pi / 2, angle_min=0.0,
                     angle_increment=math.pi / 2, count=4, range_max=10)
    # ロボットが左(+y)を向いているので、0 本目が上の壁
    assert ranges == pytest.approx([0.4, 0.9, 0.4, 0.9])


def test_bad_grid():
    with pytest.raises(ValueError):
        ls.GridMap(2, 2, 0.1, 0, 0, [0, 0, 0])
    with pytest.raises(ValueError):
        ls.GridMap(0, 2, 0.1, 0, 0, [])
