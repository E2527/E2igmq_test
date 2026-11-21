#!/usr/bin/env bash
# デスクトップとアプリのメニューに、アイコン(.desktop)を作る(ホストの PC で実行。Linux 用)
#
#   ./tools/desktop/install.sh            … 作る
#   ./tools/desktop/install.sh --remove   … 消す
#
# 作るアイコン(ダブルクリックするとターミナルが開いて起動する。止めるときはそのターミナルで Ctrl+C):
#   シミュレーター        … ./sim.sh
#   ロボットコンソール    … ./console.sh(実機を動かすとき)
#   フィールドエディタ    … field_map の field_editor
# リポジトリの場所を中に書くので、リポジトリを動かしたら、もう一度実行する。
set -euo pipefail
repo="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
apps="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
desktop="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
names=(robocon-simulator robocon-console robocon-field-editor)

if [ "${1:-}" = "--remove" ]; then
    for n in "${names[@]}"; do rm -f "$apps/$n.desktop" "$desktop/$n.desktop"; done
    echo "アイコンを消しました"
    exit 0
fi

write() {   # 名前 表示名 説明 アイコン run.sh に渡すもの
    local file="$1.desktop"
    cat > "$apps/$file" <<DESKTOP
[Desktop Entry]
Type=Application
Name=$2
Comment=$3
Icon=$4
Exec="$repo/tools/desktop/run.sh" $5
Path=$repo
Terminal=true
Categories=Development;
DESKTOP
    chmod +x "$apps/$file"
    if [ -d "$desktop" ]; then
        cp "$apps/$file" "$desktop/$file"
        chmod +x "$desktop/$file"
        # GNOME は「信頼する」の印がないと起動しない(KDE は最初に聞いてくる)
        command -v gio >/dev/null && gio set "$desktop/$file" metadata::trusted true 2>/dev/null || true
    fi
    echo "作りました: $2"
}

mkdir -p "$apps"
write robocon-simulator 'シミュレーター' '実機なしで足回りを試す(./sim.sh)' applications-science simulator
write robocon-console 'ロボットコンソール' '実機を PC から操縦する(./console.sh)' input-gaming console
write robocon-field-editor 'フィールドエディタ' 'フィールドの形を描いて編集する' applications-graphics editor
command -v update-desktop-database >/dev/null && update-desktop-database "$apps" 2>/dev/null || true
echo "デスクトップ: $desktop"
