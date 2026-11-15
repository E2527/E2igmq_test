# 隠し機能: ターミナルで Shift を押しながら(大文字で)決まった文字を打つと、文字アートの動画のウィンドウが開く。
#
# 読み込み方(どちらも、新しく開いたターミナルから効く):
#   dev コンテナ … イメージに入っているので何もしなくてよい(ros で入ったターミナルで使える)
#   ホストの PC … ~/.bashrc に次の 1 行を足す
#                   source ~/e2igmq_test/ros2_ws/egg/egg.bash
#
# 開くのは swerve_drive の ascii_player(ROS のノードではないので、ロボットの動きには関係しない)。
# ロボットコンソール(./sim.sh)からは開かない。

# 人が打つ bash のときだけ(スクリプトの中などでは何もしない)
case $- in *i*) ;; *) return 0 2>/dev/null || exit 0 ;; esac

_ROBOT_EGG_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# ビルドしなくても動くように、ソースのフォルダから直接起動する。うまく開かないときの理由は /tmp/robot_egg.log
_ROBOT_EGG_RUN='PYTHONPATH=/root/ros2_ws/src/swerve_drive${PYTHONPATH:+:$PYTHONPATH} exec python3 -m swerve_drive.ascii_player'

_robot_egg() {
    local command="${ROBOT_EGG_COMMAND:-}"
    if [ -z "$command" ]; then
        if [ -f /.dockerenv ] || [ -f /run/.containerenv ]; then
            command="$_ROBOT_EGG_RUN"
        else
            command="'$_ROBOT_EGG_REPO/dev.sh' bash -c '$_ROBOT_EGG_RUN'"
        fi
    fi
    bash -c "$command" </dev/null >/tmp/robot_egg.log 2>&1 &
    disown 2>/dev/null
}

# Shift を押しながら打つ(大文字)。最初の 1 文字を打ってから次の文字まで 1 秒待つ
# (ふつうに大文字の O を打ったときは、1 秒たつと表示される)
bind 'set keyseq-timeout 1000' 2>/dev/null
bind -x '"OMAEYARIMASUNESUGILI": _robot_egg' 2>/dev/null
