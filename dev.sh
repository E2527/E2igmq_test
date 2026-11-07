#!/usr/bin/env bash
# 開発コンテナ ros_dev を起動して入る(WSL でも Linux でも使える)
#
#   ./dev.sh              … 起動して、コンテナの bash に入る
#   ./dev.sh <コマンド>   … 起動して、コンテナでコマンドを実行する(例: ./dev.sh rviz2)
#
# どこからでも使えるように、~/.bashrc に alias ros="$HOME/e2igmq_test/dev.sh" を書いておくと便利
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"

# Linux(Fedora など)では、コンテナの root に画面への表示を許可する(WSLg では不要)
if [ -z "${WSL_DISTRO_NAME:-}" ] && [ -n "${DISPLAY:-}" ]; then
    if command -v xhost >/dev/null; then
        xhost +local: >/dev/null
    else
        echo "xhost がないので rviz2 などの GUI が表示できないかもしれません(Fedora: sudo dnf install xhost)" >&2
    fi
fi

# 止まっていれば起動する(compose.yaml を変えたときは作り直される)
docker compose up -d dev

tty=-i
if [ -t 0 ] && [ -t 1 ]; then
    tty=-it
fi
if [ $# -eq 0 ]; then
    exec docker exec "$tty" ros_dev bash
fi
# entrypoint.sh が ROS とワークスペースの setup.bash を読み込んでからコマンドを実行する
exec docker exec "$tty" ros_dev /entrypoint.sh "$@"
