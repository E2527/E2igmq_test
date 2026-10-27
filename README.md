# E2igmq_test
う、うおｗ

## ROS 2 開発環境(Docker)

ROS 2 Jazzy の開発環境を Docker で再現できるようにしています。ベースの `ros:jazzy` は amd64 / arm64 両対応なので、同じ構成が WSL(x86_64)でも Raspberry Pi(arm64)でも動きます。ただし **x86 でビルドしたイメージは Pi では動かない** ため、Pi 用は arm64 でビルドします(下記)。

| ファイル | 内容 |
|---|---|
| `docker/Dockerfile` | PC(WSL)用。`base`(土台)/ `dev`(開発用・rviz2 入り)の2ステージ |
| `docker/Dockerfile.pi` | Raspberry Pi(Pi 5 / Pi 400)用。`builder`(ビルド)/ `runtime`(ビルド結果だけを載せた軽量版)の2ステージ |
| `docker/entrypoint.sh` | ROS とワークスペースの `setup.bash` を自動で読み込む |
| `compose.yaml` | `dev` / `build` / `runtime` / `micro-ros-agent` の各サービス |
| `docker-bake.hcl` | マルチアーキテクチャビルド(`docker buildx bake`)の定義 |
| `ros2_ws/src/` | ROS 2 パッケージを置く場所 |
| `firmware/stm32_can/` | STM32 側の部品。CAN 通信とモーターの速度制御(`stm_bridge` と対になる) |
| `firmware/motor_node_f303k8/` | 上の部品を使った STM32F303K8 用の完成したプロジェクト(`make` でビルド) |
| `firmware/lib/` | ST の HAL と CMSIS(git サブモジュール。`git submodule update --init` で取得) |
| `docker/pi/80-can.network` | Pi の `can0` を自動で起動する設定 |

### 開発(WSL)

リポジトリは WSL のファイルシステム内(`~/...`)に clone してください。`/mnt/c/...` 配下はビルドが非常に遅くなります。STM32 のプロジェクトをビルドする場合は、clone のあとに `git submodule update --init` も実行します。

```bash
docker compose build dev        # 初回、または package.xml の依存を変えたとき
docker compose up -d dev        # コンテナ ros_dev を起動
docker compose exec dev bash    # コンテナに入る

# コンテナ内
colcon build --symlink-install
source install/setup.bash
```

- `ros2_ws/` はコンテナの `/root/ros2_ws` にマウントされるので、編集と git 操作は WSL 側で行います。
- ビルドとテストだけ実行して終了したいときは `docker compose run --rm build`。
- 止めるときは `docker compose stop dev`、再開は `docker compose start dev`。
- `network_mode: host` なので、同じネットワークの他マシン(Pi など)と ROS 2 トピックをやり取りできます。
- `dev` には rviz2 が入っていて、WSLg 経由で Windows に表示できます(コンテナ内で `rviz2`)。
- `dev` には MoveIt 2(アームの動作計画)も入っています。デモはコンテナ内で `ros2 launch moveit_resources_panda_moveit_config demo.launch.py`。rviz2 の「MotionPlanning」パネルで、ゴールの姿勢を動かして **Plan & Execute** を押すとアームが動きます。自分のアームの設定は `ros2 launch moveit_setup_assistant setup_assistant.launch.py` で作れます。

### Raspberry Pi(arm64)で使う

Pi では `docker/Dockerfile.pi` の `runtime` イメージを使います(Pi 5 / Pi 400 共通)。

- `ros:jazzy-ros-core` に、`ros2_ws` のビルド結果と実行時の依存(`exec_depend`)だけを載せた軽量版です。rviz2 などの GUI やコンパイラは入りません。
- root ではなく `ubuntu` ユーザー(UID 1000、`dialout` グループ所属)で動きます。Pi の最初のユーザーと UID が同じなのでファイルの持ち主がずれず、USB シリアルにもアクセスできます。
- `ros2_ws/src` のパッケージはビルド済みで焼き込まれるので、コードを変えたらイメージを作り直して送り直します。

**方法1: Pi 上で直接ビルド**(いちばん簡単。Pi 側で自動的に arm64 になります)

```bash
docker compose build runtime
STM32_DEVICE=/dev/serial/by-id/usb-STM32...  docker compose --profile robot up -d micro-ros-agent
docker compose --profile robot run --rm runtime ros2 topic list
```

**方法2: WSL で arm64 イメージを作って Pi へ送る**(Pi 400 で重いビルドを避けたいとき)

```bash
# 初回のみ: arm64 エミュレーション(QEMU)を有効化
docker run --privileged --rm tonistiigi/binfmt --install arm64

docker buildx bake -f docker-bake.hcl runtime-arm64     # robocon-ros2-arm64.tar ができる(Pi 5 / Pi 400 共通)
scp robocon-ros2-arm64.tar pi@<PiのIP>:~
ssh pi@<PiのIP> docker load -i robocon-ros2-arm64.tar
```

C++ を CPU に合わせて最適化したい場合は、`runtime-pi5`(Cortex-A76)または `runtime-pi400`(Cortex-A72)を使います。それぞれ `robocon-ros2-pi5.tar` / `robocon-ros2-pi400.tar` ができ、タグは `robocon/ros2:jazzy-pi5` / `robocon/ros2:jazzy-pi400` です。**Pi 5 向けのイメージは Pi 400 では動きません**(Pi 400 向けは Pi 5 でも動きます)。迷ったら `runtime-arm64` を使ってください。

QEMU 経由のビルドはネイティブより数倍遅くなります。開発用イメージを両アーキテクチャ分まとめて作る場合は `docker buildx bake -f docker-bake.hcl dev`(複数プラットフォームの出力にはレジストリへの push か containerd image store が必要です)。

### STM32 との通信(stm_bridge、CAN)

CAN バスにつないだ複数の STM32 と、ROS 2 のトピックを中継します。STM32 1 枚でモーター 2 個を速度制御し、**デューティ比は STM32 がエンコーダと PID で決めます**。Pi は目標速度を送るだけです。STM32 側のコードは `firmware/stm32_can/` にあります。

| トピック | 型 | 内容 |
|---|---|---|
| `stm<ID>/command` | `std_msgs/Float32MultiArray` | [モーター 1, モーター 2] の目標速度 [rad/s]。最新の値を `rate_hz` の周期で送り続ける |
| `stm<ID>/state` | `std_msgs/Float32MultiArray` | [モーター 1, モーター 2] の実際の速度 [rad/s] |

- CAN の ID は、指令が `0x100 + ID`、状態が `0x200 + ID` です(標準 11 ビット)。
- `command` が `command_timeout_s`(既定 0.5 秒)以上届かないと、空の指令(= 止まる)を送ります。STM32 側も、指令が 100 ms 届かなければ止まります。
- 設定は `ros2_ws/src/stm_bridge/config/bridge.yaml` です。5 秒ごとに ID ごとの受信数をログに出し、状態が届かない ID があれば警告します。

**実機なしで試す**(PC の `dev` コンテナ内。`sim` は UDP で CAN を真似るので、カーネルに CAN がない WSL でも動きます)

```bash
ros2 run stm_bridge fake_stm --ids 1 2 &
ros2 run stm_bridge bridge --ros-args -p can_interface:=sim -p "device_ids:=[1, 2]"
# 別のターミナルで
ros2 topic pub stm1/command std_msgs/msg/Float32MultiArray "{data: [0.5, -1.0]}"
ros2 topic echo stm1/state        # 速度が 0.1 秒ほどで [0.5, -1.0] に近づく。pub を止めると 0 に戻る
```

`fake_stm` は、速度制御されたモーターの代わりに、速度が目標に少しずつ近づくように動きます。`rqt_plot` で `stm1/command/data[0]` と `stm1/state/data[0]` を並べると、応答の様子が見えます。STM32 側のコードは `make -C /root/firmware/stm32_can/test`(dev コンテナ内)で PC でもテストできます。

**Pi で動かす**

1. USB-CAN アダプター(candleLight 互換。CANable 2.0 など)を挿すと `can0` ができます(`ip link show can0` で確認)。
2. `can0` が起動時に自動で up になるように設定します(通信速度は `docker/pi/80-can.network` の `BitRate`)。
   ```bash
   sudo cp docker/pi/80-can.network /etc/systemd/network/
   sudo systemctl restart systemd-networkd
   ip -details link show can0        # state UP、bitrate 1000000 になっていれば OK
   ```
3. ブリッジを起動します(`--network host` なので、コンテナから `can0` がそのまま使えます)。
   ```bash
   docker run -d --name stm-bridge --restart unless-stopped --network host --ipc host \
     robocon/ros2:jazzy ros2 launch stm_bridge bridge.launch.py
   ```
   リポジトリを Pi に clone している場合は `docker compose --profile robot up -d stm-bridge` でも起動できます。

**USB シリアルで動かす**(CAN の部品がないとき。Nucleo なら USB ケーブル 1 本でつながる)

STM32 を `make LINK=uart` でビルドして書き込み、ブリッジの `can_interface` を `serial:/dev/ttyACM0` にします。中身(ID とデータ)は CAN と同じで、トピックもそのままです。

```bash
ros2 run stm_bridge bridge --ros-args -p can_interface:=serial:/dev/ttyACM0
```

PC(WSL)の dev コンテナで使う場合は、先に Windows の USB を WSL に渡します(`firmware/motor_node_f303k8/README.md` の「USB シリアルで PC とつなぐ」)。dev コンテナは `/dev` を共有しているので、`/dev/ttyACM0` がそのまま見えます。
4. STM32 をつないだら、`candump can0`(`sudo apt install can-utils`)で、`101` と `201` のメッセージが流れているか確認できます。

### micro-ROS agent

`micro-ros-agent` サービスは `microros/micro-ros-agent:jazzy` を使い、`STM32_DEVICE`(既定 `/dev/ttyACM0`)のシリアルを `/dev/stm32` としてコンテナに渡します。デバイス名が変わらないよう `/dev/serial/by-id/...` を指定してください。`restart: unless-stopped` なので、agent が落ちても自動で再起動します。

### CI

`.github/workflows/docker-ros2.yml` で、プッシュと PR のたびに amd64 の開発用イメージをビルドして `colcon build` / `colcon test` を実行します。あわせて arm64 ランナーで Pi 用の `runtime` イメージがビルドできるかも確認します。
