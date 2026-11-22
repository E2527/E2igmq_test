# ROS 2 と STM32 の通信(stm_bridge)

`ros2_ws/src/stm_bridge/` は、ROS 2 のトピックと STM32 の通信(CAN か USB シリアル)を中継するノードです。足回り(swerve_drive / omni_drive)の launch が自動で起動するので、ふだんは意識しなくて大丈夫です。STM32 1 枚だけを試したいときや、通信を調べたいときに読んでください。

## トピック

STM32 の基板ごとに番号(ID。ファームウェアの `NODE_ID`)があり、ID ごとにトピックができます。

| トピック | 型 | 内容 |
|---|---|---|
| `stm<ID>/command` | `std_msgs/Float32MultiArray` | STM32 への指令。小数 2 個まで(ふつうは [モーター 1, モーター 2] の目標速度 [rad/s])。最新の値を `rate_hz` の周期で送り続ける |
| `stm<ID>/state` | `std_msgs/Float32MultiArray` | STM32 からの状態。小数 2 個まで(ふつうは実際の速度 [rad/s]) |
| `stm<ID>/param` | `std_msgs/Float32MultiArray` | STM32 の設定値(ゲインなど)の読み書き。`[番号]` で読む、`[番号, 値]` で書く |
| `stm<ID>/param_reply` | `std_msgs/Float32MultiArray` | その返事 `[番号, 結果, 今の値]`。結果は 0 = OK、1 = その基板にない、2 = 範囲の外、3 = 読むだけ、4 = 頼み方がおかしい |
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
| ブリッジ → STM32(設定値の読み書き) | `0x300 + ID` | 1 バイト `[番号]` で読む、8 バイト `[番号, 0, 0, 0, float の値]` で書く |
| STM32 → ブリッジ(その返事) | `0x400 + ID` | 8 バイト `[番号, 結果, 0, 0, float の今の値]` |

**設定値の番号**(`firmware/stm32_can/params.h` と `stm_bridge/protocol.py` の `PARAMS`。ロボットコンソールの「調整」タブが使う)

| 番号 | 中身 | あるファームウェア |
|---|---|---|
| 0 | ファームウェアの種類(読むだけ。1 = モーター 2 個、2 = スワーブ) | 全部 |
| 1〜4 | 速度の PID: kp、ki、kff、デューティ比の上限 | 全部 |
| 10〜13 | スワーブの向きの制御: kp、kd、ff、向きを変える最大の速さ | スワーブ |
| 20 | スワーブの今の向き [rad](書くと、今の向きをその値にする。0 で「今を前」) | スワーブ |

書いた値は STM32 の RAM にだけ入ります(電源を切ると `app_config.h` の値にもどる)。

**USB シリアル**(115200 bps。Nucleo の USB にそのまま出る)

```
0xAA 0x55 | CAN の ID(2 バイト) | 長さ(1 バイト) | データ | CRC16(MODBUS、2 バイト)
```

中身(ID とデータ)は CAN と同じで、外側に「始まりの目印」と「壊れていないかの確認(CRC)」を付けただけです。STM32 側のコードは `firmware/stm32_can/can_node.c` と `uart_node.c` です。

## 設定(`config/bridge.yaml`)

| 名前 | コードの初期値 | 意味 |
|---|---|---|
| `can_interface` | `can0` | 通信の線。`can0`(CAN)、`serial:/dev/ttyACM0`(USB シリアル)、`sim`(実機なしで試す) |
| `device_ids` | `[1]` | つながっている STM32 の ID の一覧 |
| `rate_hz` | `100.0` | 指令を送る周期 [Hz] |
| `command_timeout_s` | `0.5` | `stm<ID>/command` がこれ以上来なければ、空の指令(= 停止)を送る [s] |
| `state_timeout_s` | `0.5` | STM32 から状態がこれ以上届かなければ警告する [s] |
| `expected_state_hz` | `100.0` | STM32 が状態を送る回数 [回/秒]。これの半分より少ないと `/diagnostics` で「注意」にする |

ここにある初期値は `bridge_node.py` の `declare_parameter` の値です。起動時には `config/bridge.yaml` の値で上書きされることがあります。

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
