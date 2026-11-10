#!/usr/bin/env bash
# 足回りのシミュレーションを 1 コマンドで起動する(dev コンテナの中で使う。rviz2 + WASD の操縦ウィンドウ)
#
#   ./sim.sh            … スワーブ(swerve_drive)
#   ./sim.sh omni       … オムニホイール(omni_drive)
#   ./sim.sh omni joy   … WASD の代わりにゲームパッドで操縦
#
# 起動する前に、変わったところだけ colcon build する(数秒)。Ctrl+C で全部止まる。
# ホストからはリポジトリの一番上の ./sim.sh(中でこれを呼ぶ)を使う。
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"

drive=${1:-swerve}
case "$drive" in
    swerve | omni) ;;
    *)
        echo "使い方: $0 [swerve|omni] [joy]" >&2
        exit 1
        ;;
esac
if [ "${2:-}" = "joy" ]; then
    teleop="joy:=true"
else
    teleop="keyboard:=true"
fi

colcon build --symlink-install --packages-up-to "${drive}_drive"
source install/setup.bash
exec ros2 launch "${drive}_drive" "${drive}.launch.py" sim:=true rviz:=true "$teleop"
