#!/usr/bin/env bash
# 足回りのシミュレーションを 1 コマンドで起動する(dev コンテナの中で使う)
# ロボットコンソール(WASD の操縦、速さ、軌跡、状態、停止ボタンを 1 つにまとめたウィンドウ)が開く。
#
#   ./sim.sh            … スワーブ(swerve_drive)
#   ./sim.sh omni       … オムニホイール(omni_drive)
#   ./sim.sh fw         … STM32 のファームウェアもそのまま動かす(app_config.h のゲイン・ギア比が動きに出る)
#   ./sim.sh joy        … ゲームパッドでも操縦する
#   ./sim.sh rviz       … rviz2 も開く
#   (swerve / omni / fw / joy / rviz は好きな順番で組み合わせられる。例: ./sim.sh omni fw rviz)
#
# 起動する前に、変わったところだけ colcon build する(数秒)。Ctrl+C で全部止まる。
# ホストからはリポジトリの一番上の ./sim.sh(中でこれを呼ぶ)を使う。
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"

drive=swerve
firmware=false
joy=false
rviz=false
for arg in "$@"; do
    case "$arg" in
        swerve | omni) drive=$arg ;;
        fw) firmware=true ;;
        joy) joy=true ;;
        rviz) rviz=true ;;
        *)
            echo "使い方: $0 [swerve|omni] [fw] [joy] [rviz]" >&2
            exit 1
            ;;
    esac
done

colcon build --symlink-install --packages-up-to "${drive}_drive"
source install/setup.bash
exec ros2 launch "${drive}_drive" "${drive}.launch.py" sim:=true console:=true \
    "firmware:=$firmware" "joy:=$joy" "rviz:=$rviz"
