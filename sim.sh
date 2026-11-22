#!/usr/bin/env bash
# 足回りのシミュレーションを 1 コマンドで起動する(実機なし)
# シミュレーター(ロボットコンソールの機能 + ロボットを置く・障害物・わざと故障させる)が開く。
# 実機を動かすときは、こちらではなく ./console.sh(ロボットコンソール)を使う。
#
#   ./sim.sh            … スワーブ(swerve_drive)
#   ./sim.sh omni       … オムニホイール(omni_drive)
#   ./sim.sh fw         … STM32 のファームウェアもそのまま動かす
#   ./sim.sh joy        … ゲームパッドでも操縦する
#   ./sim.sh rviz       … rviz2 も開く(./sim.sh omni fw rviz のように組み合わせる)
#   ./sim.sh nav        … 自動操縦(Nav2)も起動する(「自動操縦」タブで地図をクリック)
#   ./sim.sh noamcl     … AMCL(LiDAR と地図を重ねて、ずれを定期的に直す)を使わない
#
# 起動する前に、変わったところだけ colcon build する(数秒)。Ctrl+C で全部止まる。
# コンテナの中では、~/ros2_ws で ./sim.sh(同じ使い方)を実行する。
# どこからでも使えるように、~/.bashrc に alias sim="$HOME/e2igmq_test/sim.sh" を書いておくと便利
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"

# 中身はコンテナの中の ros2_ws/sim.sh(コンテナの中では ~/ros2_ws で ./sim.sh omni のように直接使える)
exec ./dev.sh /root/ros2_ws/sim.sh "$@"
