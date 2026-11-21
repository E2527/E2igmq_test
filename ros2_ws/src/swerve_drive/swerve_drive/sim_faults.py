"""
シミュレーションで、わざと故障を起こすための設定(シミュレーターの画面から送る).

    トピック /sim/faults(std_msgs/String の JSON。あとから起動したノードにも届くように最後の値を残す)

        {"dead": [2],          … この ID の STM32 を止める(指令を受け取らず、状態も返さない)
         "nan": [3],           … この ID の STM32 の状態に NaN を混ぜる
         "drop": 0.3,          … 状態を、この割合で落とす(0〜1。通信が途切れがち)
         "slip": 0.1,          … タイヤの滑り(0〜0.9)。本当の動きがオドメトリより、この割合だけ小さい
         "drift_deg_s": 2.0,   … 本当の向きが、オドメトリより 1 秒にこれだけずれていく [度/秒]
         "lidar_noise": 0.05}  … シミュレーション用の LiDAR の距離のばらつき [m]

STM32 のふりをするノード(fake_modules など)が SimFaults で受け取って、状態を出す前に通す。
parse_faults は ROS に依存しないので、PC だけでテストできる(test/test_sim_faults.py)。
"""

import json
import math
import random

FAULT_TOPIC = '/sim/faults'
DEFAULTS = {'dead': frozenset(), 'nan': frozenset(), 'drop': 0.0, 'slip': 0.0,
            'drift_deg_s': 0.0, 'lidar_noise': None}


def parse_faults(text):
    """JSON の文字列を、決まった形の辞書にする(おかしい値は、故障なしの値にする)."""
    try:
        data = json.loads(text) if text else {}
    except ValueError:
        data = {}
    if not isinstance(data, dict):
        data = {}
    out = dict(DEFAULTS)
    for key in ('dead', 'nan'):
        try:
            out[key] = frozenset(int(v) for v in data.get(key) or [])
        except (TypeError, ValueError):
            pass
    for key, low, high in (('drop', 0.0, 1.0), ('slip', 0.0, 0.9), ('drift_deg_s', -90.0, 90.0)):
        try:
            value = float(data.get(key, out[key]))
            if math.isfinite(value):
                out[key] = min(high, max(low, value))
        except (TypeError, ValueError):
            pass
    try:
        noise = data.get('lidar_noise')
        out['lidar_noise'] = None if noise is None else max(0.0, float(noise))
    except (TypeError, ValueError):
        pass
    return out


def dump_faults(faults):
    """parse_faults の形の辞書を、JSON の文字列にする."""
    data = {k: sorted(v) if isinstance(v, frozenset) else v for k, v in faults.items()}
    return json.dumps(data)


def filter_state(faults, dev_id, values, rand=random.random):
    """STM32 の状態を出す前に通す。出さないときは None、NaN を混ぜるときは NaN の並び."""
    if dev_id in faults['dead'] or (faults['drop'] > 0 and rand() < faults['drop']):
        return None
    if dev_id in faults['nan']:
        return [math.nan] * len(values)
    return list(values)


class SimFaults:
    """ノードの中で /sim/faults を受け取って持っておく(ROS を使う部分)."""

    def __init__(self, node):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        self.faults = dict(DEFAULTS)
        self.node = node
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, FAULT_TOPIC, self.on_msg, qos)

    def on_msg(self, msg):
        old = self.faults
        self.faults = parse_faults(msg.data)
        if self.faults['dead'] != old['dead'] or self.faults['nan'] != old['nan']:
            self.node.get_logger().warn(
                f'シミュレーションの故障: 止める {sorted(self.faults["dead"]) or "なし"}、'
                f'NaN {sorted(self.faults["nan"]) or "なし"}')

    def dead(self, dev_id):
        return dev_id in self.faults['dead']

    def state(self, dev_id, values):
        return filter_state(self.faults, dev_id, values)
