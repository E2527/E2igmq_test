"""
シミュレーション用の 2D LiDAR: 地図(マス目)に向かって光を飛ばし、壁までの距離を求める.

ROS に依存しないので、PC だけでテストできる(test/test_lidar_sim.py)。

光は、マス目を 1 マスずつ順にたどる方法(DDA)で進め、ふさがっているマスに当たったら止める。
地図の外に出たら「何もない」(inf)。
"""

import math

OCCUPIED_AT = 50        # この値以上のマスを、光が当たる物とみなす


class GridMap:
    """地図のマス目(nav_msgs/OccupancyGrid と同じ並び: 左下から右へ、下の行から上へ)."""

    def __init__(self, width, height, resolution, origin_x, origin_y, cells):
        if width <= 0 or height <= 0 or resolution <= 0:
            raise ValueError('地図の大きさがおかしい')
        if len(cells) != width * height:
            raise ValueError('マスの数が、幅 × 高さと合わない')
        self.width, self.height = int(width), int(height)
        self.resolution = float(resolution)
        self.origin_x, self.origin_y = float(origin_x), float(origin_y)
        self.blocked = bytearray(1 if v >= OCCUPIED_AT else 0 for v in cells)

    def is_blocked(self, col, row):
        return self.blocked[row * self.width + col] == 1


def cast(grid, x, y, angle, range_max):
    """
    (x, y) から angle [rad] の向きに光を飛ばし、当たった物までの距離 [m] を返す.

    range_max まで何にも当たらなければ math.inf。
    """
    res = grid.resolution
    gx, gy = (x - grid.origin_x) / res, (y - grid.origin_y) / res   # マスの単位
    col, row = int(math.floor(gx)), int(math.floor(gy))
    if 0 <= col < grid.width and 0 <= row < grid.height and grid.is_blocked(col, row):
        return 0.0                              # 壁の中にいる
    dx, dy = math.cos(angle), math.sin(angle)
    step_c = 1 if dx > 0 else -1
    step_r = 1 if dy > 0 else -1
    # 次のマスの境目まで進む長さ(マスの単位)と、1 マス進むごとに増える長さ
    t_max_x = ((col + (step_c > 0)) - gx) / dx if dx != 0 else math.inf
    t_max_y = ((row + (step_r > 0)) - gy) / dy if dy != 0 else math.inf
    t_dx = abs(1 / dx) if dx != 0 else math.inf
    t_dy = abs(1 / dy) if dy != 0 else math.inf
    limit = range_max / res
    t = 0.0
    while t <= limit:
        if t_max_x < t_max_y:
            t, t_max_x = t_max_x, t_max_x + t_dx
            col += step_c
        else:
            t, t_max_y = t_max_y, t_max_y + t_dy
            row += step_r
        if not (0 <= col < grid.width and 0 <= row < grid.height):
            # 地図の外: 中に入ってくる見込みがなければ、そこで終わり
            if (col < 0 and step_c < 0) or (col >= grid.width and step_c > 0) or \
                    (row < 0 and step_r < 0) or (row >= grid.height and step_r > 0):
                return math.inf
            continue
        if grid.is_blocked(col, row):
            distance = t * res
            return distance if distance <= range_max else math.inf
    return math.inf


def scan(grid, x, y, yaw, angle_min, angle_increment, count, range_max):
    """ロボットの位置 (x, y)・向き yaw から、count 本の光の距離の並び(LaserScan の ranges と同じ)."""
    return [cast(grid, x, y, yaw + angle_min + i * angle_increment, range_max)
            for i in range(count)]
