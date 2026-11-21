#!/usr/bin/env bash
# ロボットコンソールを開く(実機を PC から動かすとき)
# 操縦・地図・LiDAR・状態・通信・グラフ・停止ボタンを 1 つにまとめたウィンドウと、ゲームパッドを起動する。
#
#   ./console.sh                … コンソール + ゲームパッド
#   ./console.sh joy:=false     … コンソールだけ(WASD で操縦)
#   ./console.sh rviz:=true     … rviz2 も開く
#
# ロボット(Pi)では足回り(docker compose --profile robot up -d drive)を起動しておく。
# PC と Pi が同じ Wi-Fi で、.env の ROS_DOMAIN_ID が同じなら、ロボットとつながる。
# 実機なしで試すときは、こちらではなく ./sim.sh(シミュレーター)を使う。
# どこからでも使えるように、~/.bashrc に alias console="$HOME/e2igmq_test/console.sh" を書いておくと便利
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"

exec ./dev.sh bash -c 'cd /root/ros2_ws && colcon build --symlink-install --packages-up-to swerve_drive >/dev/null \
    && source install/setup.bash && exec ros2 launch swerve_drive teleop.launch.py "$@"' console "$@"
