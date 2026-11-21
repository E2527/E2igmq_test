#!/usr/bin/env bash
# デスクトップのアイコンから呼ばれる(install.sh が作るアイコンの中身)
#   run.sh simulator | console | editor
# 終わったあと(エラーで止まったときも)、ターミナルがすぐ閉じないように Enter を待つ
repo="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
cd "$repo" || exit 1
case "${1:-}" in
    simulator) ./sim.sh ;;
    console) ./console.sh ;;
    editor) ./dev.sh bash -c 'cd /root/ros2_ws && colcon build --symlink-install --packages-up-to field_map >/dev/null && source install/setup.bash && exec ros2 run field_map field_editor' ;;
    *) echo "使い方: $0 simulator | console | editor" >&2; false ;;
esac
status=$?
echo
read -r -p "終わりました(Enter で閉じる)" _
exit $status
