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

### 開発(WSL)

リポジトリは WSL のファイルシステム内(`~/...`)に clone してください。`/mnt/c/...` 配下はビルドが非常に遅くなります。

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

### micro-ROS agent

`micro-ros-agent` サービスは `microros/micro-ros-agent:jazzy` を使い、`STM32_DEVICE`(既定 `/dev/ttyACM0`)のシリアルを `/dev/stm32` としてコンテナに渡します。デバイス名が変わらないよう `/dev/serial/by-id/...` を指定してください。`restart: unless-stopped` なので、agent が落ちても自動で再起動します。

### CI

`.github/workflows/docker-ros2.yml` で、プッシュと PR のたびに amd64 の開発用イメージをビルドして `colcon build` / `colcon test` を実行します。あわせて arm64 ランナーで Pi 用の `runtime` イメージがビルドできるかも確認します。
