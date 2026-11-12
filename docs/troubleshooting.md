# 困ったとき

これまでに実際に起きたトラブルと、その直し方です。上から順に、まずは**どこでコマンドを打っているか**を確認してください。

- `名前@pc名:~$` … **ホスト**(PC)。`ros`、`./sim.sh`、`git`、`docker` はここで打つ
- `root@...#` … **コンテナの中**。`ros2`、`colcon`、`make` はここで打つ(`ros` は付けない)

## コンテナ・Docker

| 症状 | 原因と直し方 |
|---|---|
| `ros: command not found` | コンテナの中で打っている → `exit` でホストに戻る。ホストなら → `source ~/.bashrc`(それでもだめならセットアップの手順 4 をやり直す) |
| `container ros_dev is not running` | コンテナが止まっている → `ros` で入り直す(自動で起動する)か、`docker compose up -d dev` |
| `docker: command not found`(コンテナの中で) | コンテナの中では docker は使えない → `exit` してホストで打つ |
| `permission denied ... docker.sock` | docker グループに入っていない → `sudo usermod -aG docker $USER` のあと、ログインし直す(WSL は PowerShell で `wsl --shutdown`) |
| rviz2 やコンソールの画面が出ない | `docker exec` で入っている → `ros` で入る(画面の許可を自動でやる)。または、ホストで `xhost +SI:localuser:root` |
| `exec format error`(Pi 用のイメージを作るとき) | PC で arm64 を動かす準備ができていない → [raspberry_pi.md](raspberry_pi.md) の「最初の 1 回だけ」 |
| ディスクがいっぱい | `docker system prune` で使っていないイメージを消す |

## ROS 2

| 症状 | 原因と直し方 |
|---|---|
| `No executable found` / `Package '...' not found` | ビルドしていない → コンテナの中で `cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash` |
| `ros2 topic pub` や `--ros-args` でエラー | 書き方の間違い → パラメータは `-p 名前:=値`(`:` ではなく `:=`)。メッセージは `"{data: [0.5, 1.0]}"` のように `"` で囲む |
| コマンドの頭に `^[[200~` が付く | 貼り付けのときの変な文字 → 消して打ち直す、または 1 行ずつ貼る |
| シミュレーションでロボットが動かない | コンソールのウィンドウをクリックしてから W A S D。コンソールの「停止」が押されていないか |
| PC と Pi でトピックが見えない | `.env` の `ROS_DOMAIN_ID` が両方で同じか、同じ Wi-Fi か、Fedora のファイアウォール([setup.md](setup.md#b-linuxfedoraubuntu-など)) |

## USB・STM32

| 症状 | 原因と直し方 |
|---|---|
| WSL で `usbipd` が見つからない | Windows 側(PowerShell)で打つ。入っていなければ `winget install usbipd` |
| `usbipd bind` で busid のエラー | busid は挿すポートで変わる → `usbipd list` で確認する。毎回やるのが面倒なら、管理者の PowerShell で `usbipd policy add --effect Allow --operation AutoBind --hardware-id 0483:374b` をしておき、`usbipd attach --wsl --hardware-id 0483:374b --auto-attach` |
| `hardware-id` が違うと言われる | Nucleo は `0483:374b`(`0438` や `043` などの打ち間違いに注意) |
| `/dev/ttyACM0` がない | WSL に USB を渡していない(上)。Linux なら USB を挿し直す |
| Fedora で USB シリアルがすぐ切れる | ModemManager が横取りしている → `sudo systemctl disable --now ModemManager` |
| 書き込めたか分からない | Nucleo の LED の意味を見る([firmware の README の「LED の意味」](../firmware/motor_node_f303k8/README.md#led-の意味)) |
| STM32CubeProgrammer で消去・書き込みに失敗する | Mode を **Under reset** にして接続 → **Full chip erase** → 「プログラミング前のフラッシュ消去をスキップする」のチェックを外して書き込む |
| `make` で何も起きない / `cp` が失敗する | 打っている場所を確認(コンテナの中で `make -C /root/firmware/motor_node_f303k8`)。`firmware/lib` が空なら、ホストで `git submodule update --init` |

## Git

| 症状 | 原因と直し方 |
|---|---|
| commit の名前やメールを間違えた | `git config --global user.name "正しい名前"` などで登録し直す。すでにした commit を直すのは難しいので、まわりの人に相談 |
| `index.lock` がある、と言われる | 途中で PC が切れた → ほかの git が動いていないのを確かめてから `rm .git/index.lock` |
| `git pull` で conflict(衝突) | 同じところを 2 人が変えた → ファイルの `<<<<<<<` 〜 `>>>>>>>` の部分を正しく直して `git add` → `git commit` |
| push で認証のエラー | パスワードではなく GitHub のトークンを使う([setup.md](setup.md#c-リポジトリとイメージの準備)) |

## それでも分からないとき

1. エラーの**全文**をコピーする(最後の数行が大事)
2. **どこで**(ホスト / コンテナ / Pi)、**何のコマンド**を打ったかをメモする
3. それを持って、先輩か Claude に聞く
