# Raspberry Pi で動かす

ロボットに載せる Raspberry Pi(Pi 5 / Pi 400)で、足回りを動かす手順です。

## しくみ

- Pi では、PC とは別の**軽いイメージ**(`docker/Dockerfile.pi` の `runtime`)を使います。rviz2 などの画面やコンパイラは入っていません。
- `ros2_ws/src` のパッケージは、**ビルドした状態でイメージに焼き込まれます**。コードや設定を変えたら、イメージを作り直して Pi に送り直します。
- PC で作ったふつうのイメージ(x86)は Pi では動きません。Pi 用(arm64)に作る必要があります。

## 1. Pi 用のイメージを作る

### 方法 1:PC で作って Pi に送る(おすすめ)

Pi でビルドすると時間がかかるので、PC で作ります。

```bash
# 最初の 1 回だけ:PC で arm64 のプログラムを動かせるようにする
docker run --privileged --rm tonistiigi/binfmt --install arm64
#   Fedora は代わりに: sudo dnf install qemu-user-static && sudo systemctl restart systemd-binfmt

cd ~/e2igmq_test
docker buildx bake -f docker-bake.hcl runtime-arm64      # robocon-ros2-arm64.tar ができる(Pi 5 / Pi 400 共通)
scp robocon-ros2-arm64.tar pi@<PiのIP>:~
ssh pi@<PiのIP> docker load -i robocon-ros2-arm64.tar
```

- `exec format error` が出たら、最初の 1 回の手順ができていません。
- CPU に合わせて速くしたいときは `runtime-pi5` / `runtime-pi400` もあります。**Pi 5 向けは Pi 400 では動きません**。迷ったら `runtime-arm64`。

### 方法 2:Pi の上で作る

Pi にもリポジトリを clone して、

```bash
docker compose build runtime
```

(Pi 400 だと時間がかかります)

## 2. PC と Pi を同じ ROS 2 のグループにする

PC と Pi の**両方**で、リポジトリの一番上に `.env` を作り、同じ番号にします(最初の 1 回だけ)。

```bash
echo "ROS_DOMAIN_ID=42" > ~/e2igmq_test/.env
```

- 大会ではほかのチームと混ざらないように、0 以外の番号にします。
- `.env` は git に入りません(`.gitignore`)。PC ごとに作ります。
- Wi-Fi でつながらないときは `ROS_STATIC_PEERS=相手のIP` も書きます(詳しくは [swerve_drive の README](../ros2_ws/src/swerve_drive/README.md#wi-fi-でつながらないとき))。
- オムニホイールにするときは `DRIVE=omni` も書きます(書かなければスワーブ)。

PC 側は、`.env` を作ったあとに `docker compose up -d dev` でコンテナを作り直します。

## 3. 足回りを起動する

Pi で(リポジトリを clone しておきます。compose.yaml と .env を使うため):

```bash
cd ~/e2igmq_test
docker compose --profile robot up -d drive                                   # CAN でつなぐとき
STM_INTERFACE=serial:/dev/ttyACM0 docker compose --profile robot up -d drive  # USB シリアルでつなぐとき
docker compose --profile robot logs -f drive                                 # ログを見る(Ctrl + C で抜ける)
```

`restart: unless-stopped` なので、Pi を再起動しても自動で起動します。止めるときは `docker compose --profile robot stop drive`。

## 4. PC から Wi-Fi で操縦する

PC で:

```bash
ros ros2 launch swerve_drive teleop.launch.py              # ロボットコンソール + ゲームパッド
ros ros2 launch swerve_drive teleop.launch.py joy:=false   # ロボットコンソールだけ(WASD)
```

詳しくは [swerve_drive の README の「実機を PC から Wi-Fi で操縦する」](../ros2_ws/src/swerve_drive/README.md#2-実機を-pc-から-wi-fi-で操縦する) を見てください。

## CAN を使うとき(Pi の設定)

1. USB-CAN アダプター(candleLight 互換。CANable 2.0 など)を挿すと `can0` ができます(`ip link show can0` で確認)。
2. `can0` が起動時に自動で使えるようにします(通信速度は `docker/pi/80-can.network` の `BitRate`)。
   ```bash
   sudo cp docker/pi/80-can.network /etc/systemd/network/
   sudo systemctl restart systemd-networkd
   ip -details link show can0        # state UP、bitrate 1000000 になっていれば OK
   ```
3. STM32 をつないで、`candump can0`(`sudo apt install can-utils`)で `101` と `201` のメッセージが流れていれば通信できています。

CAN を実機で使う前に、[STM32 のクロックの注意](../firmware/motor_node_f303k8/README.md#can-を実機で使う前にクロックの精度) も読んでください。STM32 との通信の中身は [stm_bridge.md](stm_bridge.md) にあります。

## Pi の負荷

足回り一式(swerve_drive + stm_bridge)は軽いので、Pi 400 でも余裕があります。LiDAR + Nav2 などの自動操縦を足すと重くなるので、そのときは `htop` で CPU の使用率を見ながら足していきます。
