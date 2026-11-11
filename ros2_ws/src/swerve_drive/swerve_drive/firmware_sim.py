"""
STM32 のファームウェアの計算部分(firmware/sim/firmware_sim.c)を、PC でビルドして呼ぶ.

ROS に依存しないので、PC だけでテストできる(test/test_firmware_sim.py)。
ビルドには gcc を使う(dev コンテナには入っている)。読み込むたびにビルドし直すので、
firmware/motor_node_f303k8/Core/Inc/app_config.h のゲインやギア比を変えたら、起動し直すだけで反映される。
"""

import ctypes
import os
import subprocess
import tempfile

DEFAULT_FIRMWARE_DIR = '/root/firmware'     # dev コンテナでのリポジトリの firmware/


def build(firmware_dir: str = DEFAULT_FIRMWARE_DIR) -> ctypes.CDLL:
    """firmware_sim.c とファームウェアの部品をビルドして読み込む."""
    common = os.path.join(firmware_dir, 'stm32_can')
    sources = [os.path.join(firmware_dir, 'sim', 'firmware_sim.c'),
               os.path.join(common, 'speed_pid.c'),
               os.path.join(common, 'swerve_module.c')]
    missing = [s for s in sources if not os.path.exists(s)]
    if missing:
        raise FileNotFoundError(
            f'ファームウェアのソースが見つかりません: {missing}'
            '(firmware_dir パラメータに、リポジトリの firmware/ を指定してください)')
    out = os.path.join(tempfile.mkdtemp(prefix='firmware_sim_'), 'firmware_sim.so')
    cmd = ['gcc', '-O2', '-shared', '-fPIC', '-o', out, *sources,
           '-I', common, '-I', os.path.join(firmware_dir, 'motor_node_f303k8', 'Core', 'Inc'),
           '-lm']
    try:
        subprocess.run(cmd, check=True, capture_output=True, text=True)
    except FileNotFoundError as e:
        raise RuntimeError('gcc がありません(dev コンテナの中で動かしてください)') from e
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f'ファームウェアのビルドに失敗しました:\n{e.stderr}') from e

    lib = ctypes.CDLL(out)
    f = ctypes.c_float
    lib.fw_board_create.argtypes = [f, f]
    lib.fw_board_create.restype = ctypes.c_void_p
    lib.fw_board_run.argtypes = [ctypes.c_void_p, f, f, ctypes.c_int, f, ctypes.POINTER(f)]
    lib.fw_swerve_create.argtypes = [f, f, f, f]
    lib.fw_swerve_create.restype = ctypes.c_void_p
    lib.fw_swerve_run.argtypes = [ctypes.c_void_p, f, f, ctypes.c_int, f, ctypes.POINTER(f)]
    lib.fw_destroy.argtypes = [ctypes.c_void_p]
    return lib


class _Sim:
    """C 側のシミュレーション 1 個(使い終わったら close する)."""

    def __init__(self, lib, handle, n_out):
        if not handle:
            raise MemoryError('シミュレーションを作れませんでした')
        self._lib = lib
        self._handle = handle
        self._out = (ctypes.c_float * n_out)()

    def close(self):
        if self._handle:
            self._lib.fw_destroy(self._handle)
            self._handle = None


class BoardSim(_Sim):
    """STM32 1 枚(モーター 2 個の速度制御。APP なしの通常版)."""

    def __init__(self, lib, no_load_speed=0.0, time_constant=0.0):
        """no_load_speed / time_constant が 0 なら、ファームウェアの設定に合わせた模型にする."""
        super().__init__(lib, lib.fw_board_create(no_load_speed, time_constant), 2)

    def run(self, target1, target2, running, duration):
        """指定した秒数だけ動かして、ファームウェアが Pi に返す [速さ 1, 速さ 2] を返す."""
        self._lib.fw_board_run(self._handle, target1, target2, int(running), duration, self._out)
        return [self._out[0], self._out[1]]


class SwerveSim(_Sim):
    """スワーブのモジュール 1 個(APP=swerve)."""

    def __init__(self, lib, no_load_speed=0.0, time_constant=0.0, mech_steer_ratio=0.0,
                 mech_wheel_ratio=0.0):
        """mech_* は本当の機構のギア比。0 ならファームウェアの設定(app_config.h)と同じ."""
        super().__init__(lib, lib.fw_swerve_create(no_load_speed, time_constant,
                                                   mech_steer_ratio, mech_wheel_ratio), 4)

    def run(self, target_angle, target_wheel, running, duration):
        """
        指定した秒数だけ動かして、[返す向き, 返すタイヤの速さ, 本当の向き, 本当のタイヤの速さ] を返す.

        ファームウェアのギア比の設定がまちがっていると、返す値と本当の値がずれる。
        """
        self._lib.fw_swerve_run(self._handle, target_angle, target_wheel, int(running), duration,
                                self._out)
        return list(self._out)
