# E2igmq_test
エニグマって結局見破られてドイツの敗戦につながったよな

## ROS 2 開発環境(Docker)

ROS 2 Jazzy の開発環境を Docker で再現できるようにしています。ベースの `ros:jazzy` は amd64 / arm64 両対応なので、同じ構成が WSL(x86_64)でも Raspberry Pi(arm64)でも動きます。ただし **x86 でビルドしたイメージは Pi では動かない** ため、Pi 用は arm64 でビルドします(下記)。

| ファイル | 内容 |
|---|---|
| `docker/Dockerfile` | `dev`(開発用)と `runtime`(ビルド済み・Pi 配布用)の2ステージ |
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

### Raspberry Pi(arm64)で使う

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

docker buildx bake -f docker-bake.hcl runtime-arm64     # robocon-ros2-arm64.tar ができる
scp robocon-ros2-arm64.tar pi@<PiのIP>:~
ssh pi@<PiのIP> docker load -i robocon-ros2-arm64.tar
```

QEMU 経由のビルドはネイティブより数倍遅くなります。開発用イメージを両アーキテクチャ分まとめて作る場合は `docker buildx bake -f docker-bake.hcl dev`(複数プラットフォームの出力にはレジストリへの push か containerd image store が必要です)。

### micro-ROS agent

`micro-ros-agent` サービスは `microros/micro-ros-agent:jazzy` を使い、`STM32_DEVICE`(既定 `/dev/ttyACM0`)のシリアルを `/dev/stm32` としてコンテナに渡します。デバイス名が変わらないよう `/dev/serial/by-id/...` を指定してください。`restart: unless-stopped` なので、agent が落ちても自動で再起動します。
