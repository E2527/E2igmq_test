#!/usr/bin/env bash
# 足回りのシミュレーションを 1 コマンドで起動する(dev コンテナの中で使う。rviz2 + WASD の操縦ウィンドウ + 速さモニター)
#
#   ./sim.sh            … スワーブ(swerve_drive)
#   ./sim.sh omni       … オムニホイール(omni_drive)
#   ./sim.sh omni joy   … WASD の代わりにゲームパッドで操縦
#   ./sim.sh fw         … STM32 のファームウェアもそのまま動かす(app_config.h のゲイン・ギア比が動きに出る)
#                         (swerve / omni / joy / fw は好きな順番で組み合わせられる。例: ./sim.sh omni fw)
#
# 起動する前に、変わったところだけ colcon build する(数秒)。Ctrl+C で全部止まる。
# ホストからはリポジトリの一番上の ./sim.sh(中でこれを呼ぶ)を使う。
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"

drive=swerve
teleop="keyboard:=true"
firmware="firmware:=false"
for arg in "$@"; do
    case "$arg" in
        swerve | omni) drive=$arg ;;
        joy) teleop="joy:=true" ;;
        fw) firmware="firmware:=true" ;;
        *)
            echo "使い方: $0 [swerve|omni] [joy] [fw]" >&2
            exit 1
            ;;
    esac
done

colcon build --symlink-install --packages-up-to "${drive}_drive"
source install/setup.bash
exec ros2 launch "${drive}_drive" "${drive}.launch.py" sim:=true rviz:=true monitor:=true "$teleop" "$firmware"
