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
# +local: だと同じ PC のすべてのユーザーに許可してしまうので、root だけに絞る
if [ -z "${WSL_DISTRO_NAME:-}" ] && [ -n "${DISPLAY:-}" ]; then
    if command -v xhost >/dev/null; then
        xhost +SI:localuser:root >/dev/null
    else
        echo "xhost がないので rviz2 などの GUI が表示できないかもしれません(Fedora: sudo dnf install xhost)" >&2
    fi
fi

# 音: Linux では、コンテナから PC の PipeWire / PulseAudio に音を送る(WSLg は compose.yaml の既定のままでよい)
pulse_socket=""
if [ -z "${WSL_DISTRO_NAME:-}" ]; then
    uid=$(id -u)
    pulse_socket="/run/user/$uid/pulse/native"
    if [ "$uid" != 1000 ]; then
        export CONTAINER_PULSE_SERVER="unix:/run/host-user/$uid/pulse/native"
    fi
fi

# 止まっていれば起動する(compose.yaml を変えたときは作り直される)
docker compose up -d dev

# PC を起動してすぐ(ログインする前)にコンテナが起動していると、あとからできた音の口がコンテナから見えない。
# そのときは、コンテナの中で何も動いていなければ再起動して見えるようにする
if [ -n "$pulse_socket" ] && [ -S "$pulse_socket" ] \
        && ! docker exec ros_dev test -S "/run/host-user/${pulse_socket#/run/user/}"; then
    if [ "$(docker top ros_dev -o pid | tail -n +2 | wc -l)" -le 1 ]; then
        docker restart ros_dev >/dev/null
    else
        echo "音が出ないときは、ほかのターミナルで動いているものを止めてから docker restart ros_dev" >&2
    fi
fi

tty=-i
if [ -t 0 ] && [ -t 1 ]; then
    tty=-it
fi
if [ $# -eq 0 ]; then
    exec docker exec "$tty" ros_dev bash
fi
# entrypoint.sh が ROS とワークスペースの setup.bash を読み込んでからコマンドを実行する
exec docker exec "$tty" ros_dev /entrypoint.sh "$@"
