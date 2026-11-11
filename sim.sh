#!/usr/bin/env bash
# 足回りのシミュレーションを 1 コマンドで起動する(実機なし)
# ロボットコンソール(WASD の操縦、速さ、軌跡、状態、停止ボタンを 1 つにまとめたウィンドウ)が開く。
#
#   ./sim.sh            … スワーブ(swerve_drive)
#   ./sim.sh omni       … オムニホイール(omni_drive)
#   ./sim.sh fw         … STM32 のファームウェアもそのまま動かす
#   ./sim.sh joy        … ゲームパッドでも操縦する
#   ./sim.sh rviz       … rviz2 も開く(./sim.sh omni fw rviz のように組み合わせる)
#
# 起動する前に、変わったところだけ colcon build する(数秒)。Ctrl+C で全部止まる。
# コンテナの中では、~/ros2_ws で ./sim.sh(同じ使い方)を実行する。
# どこからでも使えるように、~/.bashrc に alias sim="$HOME/e2igmq_test/sim.sh" を書いておくと便利
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"

# 中身はコンテナの中の ros2_ws/sim.sh(コンテナの中では ~/ros2_ws で ./sim.sh omni のように直接使える)
exec ./dev.sh /root/ros2_ws/sim.sh "$@"
