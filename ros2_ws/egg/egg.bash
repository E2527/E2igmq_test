# 隠し機能: ターミナルで Shift を押しながら(大文字で)決まった文字を打つと、文字アートの動画のウィンドウが開く。
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

# Shift を押しながら打つ(大文字)。最初の 1 文字を打ってから次の文字まで 1 秒待つ
# (ふつうに大文字の O を打ったときは、1 秒たつと表示される)
bind 'set keyseq-timeout 1000' 2>/dev/null
bind -x '"OMAEYARIMASUNESUGILI": _robot_egg' 2>/dev/null
