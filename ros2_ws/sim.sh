#!/usr/bin/env bash
# 足回りのシミュレーションを 1 コマンドで起動する(dev コンテナの中で使う)
# シミュレーター(ロボットコンソールの機能 + ロボットを置く・障害物・わざと故障させる)が開く。
# 実機を動かすときは、こちらではなく ./console.sh(ロボットコンソール)を使う。
#
#   ./sim.sh            … スワーブ(swerve_drive)
#   ./sim.sh omni       … オムニホイール(omni_drive)
#   ./sim.sh fw         … STM32 のファームウェアもそのまま動かす(app_config.h のゲイン・ギア比が動きに出る)
#   ./sim.sh joy        … ゲームパッドでも操縦する
#   ./sim.sh rviz       … rviz2 も開く
#   ./sim.sh nav        … 自動操縦(Nav2)も起動する。シミュレーターの「自動操縦」タブで地図をクリックすると走る
#   ./sim.sh amcl       … 自動操縦 + AMCL(LiDAR と地図を重ねて自分の位置を合わせる。実機と同じやり方)
#   (swerve / omni / fw / joy / rviz / nav / amcl は好きな順番で組み合わせられる。例: ./sim.sh omni nav)
#
# 起動する前に、変わったところだけ colcon build する(数秒)。Ctrl+C で全部止まる。
# ホストからはリポジトリの一番上の ./sim.sh(中でこれを呼ぶ)を使う。
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"

drive=swerve
firmware=false
joy=false
rviz=false
nav=false
amcl=false
for arg in "$@"; do
    case "$arg" in
        swerve | omni) drive=$arg ;;
        fw) firmware=true ;;
        joy) joy=true ;;
        rviz) rviz=true ;;
        nav) nav=true ;;
        amcl) nav=true amcl=true ;;
        *)
            echo "使い方: $0 [swerve|omni] [fw] [joy] [rviz] [nav] [amcl]" >&2
            exit 1
            ;;
    esac
done

colcon build --symlink-install --packages-up-to "${drive}_drive" robot_nav
source install/setup.bash
exec ros2 launch "${drive}_drive" "${drive}.launch.py" sim:=true console:=true \
    "firmware:=$firmware" "joy:=$joy" "rviz:=$rviz" "nav:=$nav" "amcl:=$amcl"
