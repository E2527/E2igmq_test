# 隠し機能: ターミナルで Ctrl を押しながら決まった文字を打つと、文字アートの動画のウィンドウが開く。
#
# 読み込み方(どちらも、新しく開いたターミナルから効く):
#   dev コンテナ … イメージに入っているので何もしなくてよい(ros で入ったターミナルで使える)
#   ホストの PC … ~/.bashrc に次の 1 行を足す
#                   source ~/e2igmq_test/ros2_ws/egg/egg.bash
#
# 開くのは ros2 run swerve_drive ascii_player(ROS のノードではないので、ロボットの動きには関係しない)。
# ロボットコンソール(./sim.sh)からは開かない。

# 人が打つ bash のときだけ(スクリプトの中などでは何もしない)
case $- in *i*) ;; *) return 0 2>/dev/null || exit 0 ;; esac

_ROBOT_EGG_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

_robot_egg() {
    if [ -n "${ROBOT_EGG_COMMAND:-}" ]; then
        bash -c "$ROBOT_EGG_COMMAND" </dev/null >/dev/null 2>&1 &
    elif [ -f /.dockerenv ] || [ -f /run/.containerenv ]; then
        ros2 run swerve_drive ascii_player </dev/null >/dev/null 2>&1 &
    else
        "$_ROBOT_EGG_REPO/dev.sh" ros2 run swerve_drive ascii_player </dev/null >/dev/null 2>&1 &
    fi
    disown 2>/dev/null
}

# Ctrl+S は、ふつうは端末の画面を止める(Ctrl+Q で戻る)機能に使われているので、それを切る
if [ -t 0 ]; then
    stty -ixon 2>/dev/null
fi
# Ctrl+O(1 つ前の履歴を実行して次を出す。ふだんはほぼ使わない)を外して、
# 最初の文字を打ったあとに時間切れにならないようにする
bind -r '\C-o' 2>/dev/null
bind -x '"\C-o\C-m\C-a\C-e\C-y\C-a\C-r\C-i\C-m\C-a\C-s\C-u\C-n\C-e\C-s\C-u\C-g\C-i\C-l\C-i": _robot_egg' 2>/dev/null
