# 開発環境のセットアップ

自分の PC に、このリポジトリの開発環境を作る手順です。**最初の 1 回だけ**やります(30 分〜1 時間。イメージを作るのに時間がかかります)。

開発環境は Docker の中に入っているので、PC に ROS 2 を直接入れる必要はありません。PC に入れるのは **Docker と Git** だけです。

- Windows の人 → [A. Windows(WSL)](#a-windowswsl)
- Linux(Fedora、Ubuntu など)の人 → [B. Linux](#b-linuxfedoraubuntu-など)
- どちらも、そのあと [C. リポジトリとイメージの準備](#c-リポジトリとイメージの準備) へ

---

## A. Windows(WSL)

Windows の中で Linux(Ubuntu)を動かす **WSL** を使います。

**PowerShell を「管理者として実行」で開いて:**

```powershell
wsl --install -d Ubuntu-24.04      # 終わったら PC を再起動
winget install usbipd              # Nucleo(STM32)の USB を WSL に渡すため
usbipd policy add --effect Allow --operation AutoBind --hardware-id 0483:374b
```

再起動したら、スタートメニューから **Ubuntu** を開きます(最初にユーザー名とパスワードを決めます)。ここから先は **Ubuntu のターミナル**で作業します。

```bash
# Docker を入れる(Docker Desktop は使わない)
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER
```

ここで一度、**PowerShell で `wsl --shutdown`** を実行してから、Ubuntu を開き直します(docker を sudo なしで使えるようにするため)。

- STM32 に書き込むときに Windows の **STM32CubeProgrammer** を使う場合は、入れておきます(Linux なら不要)。
- VS Code を使うなら、拡張機能「WSL」「Dev Containers」を入れておくと便利です。

**注意**:リポジトリは必ず **Ubuntu の中(`~/` の下)**に置きます。`/mnt/c/...`(Windows のフォルダ)に置くと、とても遅くなります。

## B. Linux(Fedora、Ubuntu など)

```bash
curl -fsSL https://get.docker.com | sudo sh
sudo systemctl enable --now docker
sudo usermod -aG docker $USER      # このあと、ログアウトしてログインし直す
```

- Fedora では、最初から入っている Podman ではなく Docker を使います(compose の設定がそのまま動くため)。
- Linux では WSL も usbipd もいりません。Nucleo を USB でつなぐと、コンテナからそのまま見えます。

Fedora で気をつけること:

| 症状 | 対処 |
|---|---|
| USB シリアル(`/dev/ttyACM0`)がすぐ切れる | `sudo systemctl disable --now ModemManager`(USB シリアルを横取りするため。携帯回線のモデムを使っていなければ不要なサービス) |
| Pi と ROS 2 で通信できない | ファイアウォールで、Pi とつなぐネットワークを許可する(例:`sudo firewall-cmd --permanent --zone=trusted --add-source=192.168.0.0/24 && sudo firewall-cmd --reload`。アドレスは自分のネットワークに合わせる) |
| SELinux で拒否される | dev コンテナは `label=disable` にしてあるので、ふつうは起きない |

---

## C. リポジトリとイメージの準備

ここからは WSL でも Linux でも同じです(**ホストのターミナル**で)。

```bash
# 1. Git に自分の名前とメールを登録(commit に記録される)
git config --global user.name "名前"
git config --global user.email "メールアドレス"

# 2. リポジトリを取ってくる
git clone https://github.com/E2527/E2igmq_test.git ~/e2igmq_test
cd ~/e2igmq_test
git submodule update --init          # STM32 の HAL(firmware/lib)

# 3. 開発環境のイメージを作る(初回は 20〜40 分)
docker compose build dev

# 4. ros(コンテナに入る)と sim(シミュレーション)のコマンドを登録
echo 'alias ros="$HOME/e2igmq_test/dev.sh"' >> ~/.bashrc
echo 'alias sim="$HOME/e2igmq_test/sim.sh"' >> ~/.bashrc
source ~/.bashrc
```

GitHub のユーザー名とパスワードを聞かれたら、パスワードの代わりに**トークン**(GitHub の Settings → Developer settings → Personal access tokens で作る)を入れます。

### 動作確認

```bash
sim
```

少し待つと「ロボットコンソール」のウィンドウが開きます。ウィンドウをクリックして **W** を押し、ロボットが前に進めば成功です 🎉 止めるときは、ターミナルで **Ctrl + C** です。

うまくいかないときは [troubleshooting.md](troubleshooting.md) を見てください。

---

## Docker の使い方(覚えておくと便利)

ふだんは `ros` と `sim` だけで足ります。中で何をしているか知りたいときは、ここを見てください。

| コマンド | 意味 |
|---|---|
| `ros` | コンテナが止まっていれば起動して、中に入る(`./dev.sh`)。Linux では画面への表示の許可(`xhost`)も自動でやる |
| `ros コマンド` | コンテナの中で、そのコマンドだけ実行する。例:`ros ros2 topic list` |
| `sim` / `sim omni fw` など | ビルドして、シミュレーションを起動する(`./sim.sh`) |
| `docker compose build dev` | イメージを作り直す。**Dockerfile が変わったときだけ**必要(PR などで知らされる) |
| `docker compose up -d dev` | コンテナを作り直して起動する(compose.yaml が変わったとき) |
| `docker compose run --rm build` | ビルドとテストだけをして終わる(CI と同じ) |
| `docker compose stop dev` | コンテナを止める |

- `ros2_ws/` と `firmware/` は、**PC とコンテナで同じフォルダ**です(マウント)。ファイルの編集と git の操作は PC 側でやり、ビルドと実行はコンテナの中でやります。
- コンテナの中(`root@...#`)では、`ros` を付けずにそのまま `ros2 ...` や `colcon build` を打ちます。
- 新しいノード(コマンド)が増えたときは、コンテナの中で `cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash` を実行します(`sim` は自動でやる)。

## dev イメージに入っているもの

| もの | 用途 | 試し方(コンテナの中で) |
|---|---|---|
| ROS 2 Jazzy、rviz2 | 基本 | `rviz2` |
| ゲームパッド、キーボードで操縦する部品 | 操縦 | `sim joy` |
| PyQt5、日本語フォント | ロボットコンソール、設定画面 | `sim` |
| rqt_robot_monitor | `/diagnostics` を標準のツールで見る | `ros2 run rqt_robot_monitor rqt_robot_monitor` |
| ARM 用の gcc、openocd | STM32 のビルドと書き込み | `make -C /root/firmware/motor_node_f303k8` |
| MoveIt 2 | アームの動作計画 | `ros2 launch moveit_resources_panda_moveit_config demo.launch.py`(rviz2 の MotionPlanning で Plan & Execute) |
| Nav2、slam_toolbox | 自動操縦、地図づくり | `ros2 launch nav2_bringup tb3_simulation_launch.py`(rviz2 で 2D Pose Estimate → Nav2 Goal) |

## 新しい PC に引っ越すとき

全部 GitHub に入っているので、新しい PC で上の手順をやり直すだけです。古い PC からフォルダをコピーするより確実です。古い PC に push していない変更があれば、先に `git status` で確認して push しておきます。
