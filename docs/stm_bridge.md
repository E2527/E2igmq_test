# ROS 2 と STM32 の通信(stm_bridge)

`ros2_ws/src/stm_bridge/` は、ROS 2 のトピックと STM32 の通信(CAN か USB シリアル)を中継するノードです。足回り(swerve_drive / omni_drive)の launch が自動で起動するので、ふだんは意識しなくて大丈夫です。STM32 1 枚だけを試したいときや、通信を調べたいときに読んでください。

## トピック

STM32 の基板ごとに番号(ID。ファームウェアの `NODE_ID`)があり、ID ごとにトピックができます。

| トピック | 型 | 内容 |
|---|---|---|
| `stm<ID>/command` | `std_msgs/Float32MultiArray` | STM32 への指令。小数 2 個まで(ふつうは [モーター 1, モーター 2] の目標速度 [rad/s])。最新の値を `rate_hz` の周期で送り続ける |
| `stm<ID>/state` | `std_msgs/Float32MultiArray` | STM32 からの状態。小数 2 個まで(ふつうは実際の速度 [rad/s]) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | ID ごとに、状態が届いているか(ロボットコンソールの「状態」に出る) |

スワーブ(`APP=swerve`)のときは、中身が [向き, タイヤの速さ] になります(`firmware/motor_node_f303k8/README.md` の「スワーブのモジュール」)。

## 安全のしくみ

- `stm<ID>/command` が `command_timeout_s`(0.5 秒)以上来ないと、ブリッジは「止まれ」を送ります。
- STM32 側も、指令が 100 ms 届かなければモーターを止めます。
- つまり、PC や Pi のプログラムが落ちたり、通信が切れたりしても、ロボットは止まります。

## 通信の中身

ROS 2 側と STM32 側で、同じ決まりを使っています(変えるときは両方そろえる)。

**CAN**(1 Mbps、標準 11 ビット ID)

| 向き | CAN の ID | データ |
|---|---|---|
| ブリッジ → STM32(指令) | `0x100 + ID` | float 2 個(8 バイト、リトルエンディアン) |
| STM32 → ブリッジ(状態) | `0x200 + ID` | float 2 個(8 バイト) |

**USB シリアル**(115200 bps。Nucleo の USB にそのまま出る)

```
0xAA 0x55 | CAN の ID(2 バイト) | 長さ(1 バイト) | データ | CRC16(MODBUS、2 バイト)
```

中身(ID とデータ)は CAN と同じで、外側に「始まりの目印」と「壊れていないかの確認(CRC)」を付けただけです。STM32 側のコードは `firmware/stm32_can/can_node.c` と `uart_node.c` です。

## 設定(`config/bridge.yaml`)

| 名前 | 意味 |
|---|---|
| `can_interface` | `can0`(CAN)、`serial:/dev/ttyACM0`(USB シリアル)、`sim`(実機なしで試す) |
| `device_ids` | つながっている STM32 の ID の一覧(例:`[1, 2, 3, 4]`) |
| `rate_hz` | 指令を送る周期(100 Hz) |
| `command_timeout_s` / `state_timeout_s` | 指令が来ないとき止める時間 / 状態が来ないとき警告する時間 |

起動するときに `--ros-args -p 名前:=値` で上書きできます(`:` ではなく **`:=`**)。

## 試し方

### 実機なしで(PC のコンテナの中)

`sim` は UDP で CAN を真似るので、CAN がない WSL でも動きます。`fake_stm` が STM32 の代わりをします。

```bash
ros2 run stm_bridge fake_stm --ids 1 2 &
ros2 run stm_bridge bridge --ros-args -p can_interface:=sim -p "device_ids:=[1, 2]"
# 別のターミナルで
ros2 topic pub stm1/command std_msgs/msg/Float32MultiArray "{data: [0.5, -1.0]}"
ros2 topic echo stm1/state        # 0.1 秒ほどで [0.5, -1.0] に近づく。pub を止めると 0 に戻る
```

`rqt_plot` で `stm1/command/data[0]` と `stm1/state/data[0]` を並べると、応答の様子がグラフで見えます。

### Nucleo を USB でつないで(PC)

STM32 を `make LINK=uart` でビルドして書き込みます(書き込み方は [firmware の README](../firmware/motor_node_f303k8/README.md))。WSL の人は、先に USB を WSL に渡します(同じ README の「USB シリアルで PC とつなぐ」)。

```bash
ros2 run stm_bridge bridge --ros-args -p can_interface:=serial:/dev/ttyACM0
```

### Pi で、STM32 1 枚だけ

```bash
docker compose --profile bridge up -d stm-bridge                                    # CAN
STM_INTERFACE=serial:/dev/ttyACM0 docker compose --profile bridge up -d stm-bridge  # USB シリアル
```

足回りの `drive` と同時に起動すると、同じ STM32 を取り合うので、どちらか片方だけにします。CAN の準備は [raspberry_pi.md](raspberry_pi.md#can-を使うときpi-の設定) にあります。

## micro-ROS agent(今は使っていない)

STM32 で micro-ROS を動かす方法もありますが、今は stm_bridge を使っているので、ふだんは起動しません(同じ `/dev/ttyACM0` を取り合うため)。使うときは:

```bash
docker compose --profile micro-ros up -d micro-ros-agent
```

`STM32_DEVICE`(既定 `/dev/ttyACM0`)のシリアルを使います。デバイス名が変わらないように `/dev/serial/by-id/...` を指定するのがおすすめです。
