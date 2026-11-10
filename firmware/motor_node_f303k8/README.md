# STM32F303K8 モーター制御ノード

Nucleo-F303K8(STM32F303K8)用の、そのままビルドして書き込めるプロジェクトです。CAN(または USB シリアル)で Pi / PC の `stm_bridge` から目標速度を受け取り、エンコーダ付き DC モーター 2 個を PID で速度制御して、実際の速度を返します。

- 通信と制御の中身は `../stm32_can/`(`can_node`、`motor`、`speed_pid`)を使います。
- ST の HAL と CMSIS は `../lib/` に git サブモジュールとして入っています。
- CubeMX が生成するのと同じ形で初期化を書いてあるので、CubeIDE なしで `make` だけでビルドできます。

## ピンの割り当て

| 機能 | ピン | Nucleo-F303K8 | つなぐ先 |
|---|---|---|---|
| CAN_RX / CAN_TX | PA11 / PA12 | D10 / D2 | CAN トランシーバーの R / D |
| モーター1 PWM | PA8 | D9 | モータードライバー1 の PWM |
| モーター2 PWM | PA9 | D1 | モータードライバー2 の PWM |
| モーター1 方向 | PB0 | D3 | モータードライバー1 の DIR(PHASE) |
| モーター2 方向 | PB1 | D6 | モータードライバー2 の DIR(PHASE) |
| エンコーダ1 A / B | PA0 / PA1 | A0 / A1 | エンコーダ1 |
| エンコーダ2 A / B | PB4 / PB5 | D12 / D11 | エンコーダ2 |
| 状態 LED | PB3 | D13(LD3、基板上の緑 LED) | — |
| 外付け LED | PA4 | A3 | `make APP=led` のときに点滅させる LED |
| USB シリアル TX / RX | PA2 / PA15 | (基板の中で ST-LINK につながっている) | `make LINK=uart` のときの通信 |

- Nucleo の表記(D9 など)は、念のため UM1956(Nucleo-32 のユーザーマニュアル)のピン配置図で確認してください。
- エンコーダの入力は内部でプルアップしています。5 V 出力のエンコーダは、つなぐ前にピンが 5 V に耐えられるか(データシートの FT)確認するか、レベル変換を入れてください。
- PWM は 20 kHz です。モータードライバーは「PWM 1 本 + 方向ピン」で動かすものを想定しています。

## ビルド

最初に一度だけ、HAL(サブモジュール)を取ってきます(WSL で、リポジトリの一番上で)。

```bash
cd ~/e2igmq_test
git submodule update --init
```

ビルドは dev コンテナの中で行います(ARM 用のコンパイラが入っています)。

```bash
docker exec -it ros_dev bash
cd /root/firmware/motor_node_f303k8
make -j4                  # ID 1 の基板用 → build/node1/motor_node.elf(.hex、.bin も)
make NODE_ID=2 -j4        # ID 2 の基板用 → build/node2/
make TEST_MODE=1 -j4      # CAN なしで回る配線確認用 → build/node1-test/
make LINK=uart -j4        # CAN の代わりに USB シリアルで通信 → build/node1-uart/
make LINK=uart APP=led -j4  # 送った値で LED を点滅させるデモ → build/node1-uart-led/
make APP=swerve NODE_ID=1 -j4  # スワーブのモジュール(左前)→ build/node1-swerve/
```

最後に Flash と RAM の使用量が表示されます(Flash 約 14〜20 KB / 64 KB、RAM 約 2 KB / 12 KB)。

## 書き込み

Windows の **STM32CubeProgrammer** を使います(CubeIDE を入れていれば一緒に入っています)。

1. Nucleo を USB で PC につなぎます。
2. CubeProgrammer の右上で「ST-LINK」を選び、**Connect** を押します。
3. 左の「Erasing & programming」で、ファイルに次を指定します(エクスプローラーのアドレス欄に貼り付けて開けます)。
   ```
   \\wsl$\Ubuntu\home\souta\e2igmq_test\firmware\motor_node_f303k8\build\node1\motor_node.elf
   ```
4. **Start Programming** を押します。終わったら Disconnect して、Nucleo のリセットボタンを押します。

### コンテナの中から書き込む(`make flash`)

dev コンテナには openocd が入っているので、Nucleo が USB で見えていれば(Linux の PC ならそのまま、WSL なら usbipd で attach したあと)、ビルドと書き込みを 1 行でできます。CubeProgrammer も detach もいりません。

```bash
make LINK=uart APP=led flash      # ビルドして書き込む(ビルドの指定は make と同じ)
```

最後に `** Verified OK **` と出れば成功です。`Error: open failed` は ST-LINK が見えていない(USB をつなぐ / attach する)、消去や書き込みに失敗するときは `make flash OPENOCD_RESET=connect_assert_srst`(リセットしながらつなぐ)を試します。

## USB シリアルで PC とつなぐ(CAN の部品がないとき)

Nucleo の USB には ST-LINK が入っていて、書き込みだけでなく USB シリアル(仮想 COM ポート)にもなります。STM32 の USART2(PA2 / PA15)がこれにつながっているので、**USB ケーブル 1 本で**、CAN と同じ中身の通信ができます。

```
PC(WSL の dev コンテナ)── USB ──▶ ST-LINK(/dev/ttyACM0)── USART2 ──▶ STM32
```

1. **USB シリアル版を書き込む**:`make LINK=uart -j4` でビルドし、`build/node1-uart/motor_node.elf` を CubeProgrammer で書き込みます。LD3 が速く点滅(指令待ち)すれば OK です。
2. **Windows の USB を WSL に渡す**(usbipd-win。Windows の PowerShell で)
   ```powershell
   winget install usbipd                # 初回だけ。インストール後に PowerShell を開き直す
   usbipd list                                      # 「ST-Link」(0483:374b)の行があるか確認
   usbipd bind --hardware-id 0483:374b              # 初回だけ。管理者の PowerShell で
   usbipd attach --wsl --hardware-id 0483:374b      # 挿し直すたびに実行(WSL を開いておく)
   ```
   - BUSID(`1-4` など)で指定してもよいですが、ST-LINK はつなぎ直すと番号が変わり「There is no device with busid」になることがあるので、ID(`0483:374b`)で指定するほうが確実です。
   - `usbipd list` の ST-Link の行が `Attached` になり、WSL で `ls /dev/ttyACM*` を実行して `/dev/ttyACM0` が出れば OK です。
3. **dev コンテナでブリッジを起動する**(dev コンテナは `/dev` を共有しているので、そのまま見えます)
   ```bash
   ros2 run stm_bridge bridge --ros-args -p can_interface:=serial:/dev/ttyACM0
   ```
   `can_interface:=` の `=` を忘れると「Couldn't parse parameter override rule」になります。5 秒ごとに `状態の受信数(5 秒間) stm1: 500` と出れば、STM からの状態が 100 Hz で届いています(0 なら USB シリアル版が書き込まれていない)。

   別のターミナルで指令を送ると、LD3 が「ゆっくり点滅」に変わり、`stm1/state` に速度が返ってきます。
   ```bash
   ros2 topic pub -r 20 stm1/command std_msgs/msg/Float32MultiArray "{data: [1.0, 1.0]}"
   ros2 topic echo stm1/state
   ```

**注意**:USB を WSL に渡している間は、Windows の CubeProgrammer から ST-LINK が見えません。書き込むときは、先に PowerShell で `usbipd detach --hardware-id 0483:374b` を実行します。

## LED デモ:送った値で LED を点滅させる(`make APP=led`)

モーターの代わりに、ROS 2 から送った値 [Hz] の速さで、外付けの LED を点滅させます。通信を試すのにちょうどよいデモです。

**配線**:`A3(PA4)── 330 Ω の抵抗 ── LED の長い足(+)`、`LED の短い足(−)── GND`

1. `make LINK=uart APP=led -j4` でビルドし、`build/node1-uart-led/motor_node.elf` を書き込みます(書き込む前に `usbipd detach`、終わったら `usbipd attach`)。
2. ブリッジを起動します(USB シリアルのときと同じ)。
3. 別のターミナルで、点滅の速さ [Hz] を送ります。
   ```bash
   ros2 topic pub -r 10 stm1/command std_msgs/msg/Float32MultiArray "{data: [2.0]}"   # 1 秒に 2 回
   ros2 topic pub -r 10 stm1/command std_msgs/msg/Float32MultiArray "{data: [10.0]}"  # 速く
   ros2 topic pub -r 10 stm1/command std_msgs/msg/Float32MultiArray "{data: [0.0]}"   # 消灯
   ```
   - 0 以下で消灯、50 Hz より大きい値は 50 Hz になります。
   - 送るのを止めると(Ctrl+C)、0.5 秒ほどで消灯します。`--once` で 1 回だけ送っても、すぐに消えます。
   - `ros2 topic echo stm1/state` で `[使っている速さ, LED(1 = 点灯、0 = 消灯)]` が返ってきます。

## オムニホイールで使う(通常版のまま)

オムニホイールの足回り(Pi の `omni_drive`)では、**この通常版(`APP` なし)をそのまま使います**。
オムニホイールは向きを変える動きがないので、STM32 は「タイヤ 2 個を、言われた速さで回す」だけです。どのタイヤをどの速さで回すかは、Pi の `omni_drive` が計算します。

```
Pi(omni_drive)─ stm1/command [ホイールの速さ × 2] ─▶ STM32 ID 1(通常版)─▶ モーター × 2(速度 PID)
                ─ stm2/command [ホイールの速さ × 2] ─▶ STM32 ID 2(通常版)─▶ モーター × 2(速度 PID)
```

| 足回り | 基板 | ビルド |
|---|---|---|
| 4 輪 | 2 枚(1 枚でホイール 2 個) | `make NODE_ID=1`、`make NODE_ID=2` |
| 3 輪 | 2 枚(1 枚はモーター 1 個だけ使う) | 同上 |

- どのホイールをどの基板のどのモーター(1 か 2)につなぐかは、Pi の `ros2_ws/src/omni_drive/config/omni.yaml`(`device_ids` と `channels`)で決めます。
- 速度は**タイヤの軸**で測るので、`APP_COUNTS_PER_REV` は「タイヤ 1 回転あたりのカウント数」(エンコーダの PPR × 4 × 減速比)にします。
- 初めて動かすときは、下の「初めて動かすときの順番」のとおり、`TEST_MODE=1` で 2 個のモーターが前向きに回るか、`APP_MOTOR1_DIR_SIGN` などで合わせます。そのあと、Pi で `omni_drive` を起動して W を押し、ロボットが前に進む向きに回らないタイヤは、Pi の `omni.yaml` の `wheel_sign` で `-1` にします(STM32 を書き換えなくてよい)。

## スワーブのモジュール(`make APP=swerve`)

差動式(同軸差動ギア)の独立ステアリングのモジュール 1 個を、STM32 1 枚で動かします。Pi の `swerve_drive`(`ros2_ws/src/swerve_drive`)と組み合わせて使います。

```
Pi ──[目標の向き rad, タイヤの速さ rad/s]──▶ STM32 ─┬─ 向きの制御 ─┐
                                                      └─ タイヤ ─────┴─ 差動ミキシング ─▶ モーター1、モーター2(速度 PID)
   ◀─[今の向き rad, 今のタイヤの速さ rad/s]──
```

- **2 個のモーターが同じ向きに回るとモジュールが回り、逆向きに回るとタイヤが転がる**機構を想定しています。
- 向きは、ずれに比例した速さで、近いほうの回り方で合わせます。向きがずれている間はタイヤを遅くし、モーターの上限を超えるときは向きを優先します。
- 計算の中身は `../stm32_can/swerve_module.c`(PC でテストできる)、アプリ本体は `Core/Src/app_swerve.c` です。

**ID**:モジュールごとに `NODE_ID` を変えて書き込みます(Pi の `config/swerve.yaml` の `device_ids` と合わせる)。

| モジュール | ビルド |
|---|---|
| 左前 | `make APP=swerve NODE_ID=1` |
| 右前 | `make APP=swerve NODE_ID=2` |
| 左後 | `make APP=swerve NODE_ID=3` |
| 右後 | `make APP=swerve NODE_ID=4` |

**向きの基準**:今は、**電源を入れた(リセットした)ときの向きを「まっすぐ前」**とします。起動する前に、モジュールを手でまっすぐ前に向けてください。絶対角度センサーを付けたら、`app_swerve.c` の `update_module_angle()` でセンサーを読むように書き換えます(Pi 側は変えなくてよい)。

**機構に合わせる設定**(`Core/Inc/app_config.h`)

| 設定 | 中身 |
|---|---|
| `APP_COUNTS_PER_REV` | **差動ギアに入る軸** 1 回転あたりのエンコーダのカウント数(モーターの軸ではなく、減速したあと) |
| `APP_SWERVE_STEER_RATIO` | 2 個のモーターが同じ向きに 1 rad 回ると、モジュールが何 rad 回るか |
| `APP_SWERVE_WHEEL_RATIO` | 2 個のモーターが逆向きに 1 rad ずつ回ると、タイヤが何 rad 回るか |
| `APP_SWERVE_COUPLING` | モジュールだけを回したときに、タイヤが一緒に回る割合(ふつうは 0) |
| `APP_SWERVE_ANGLE_KP` / `APP_SWERVE_MAX_STEER_RATE` | 向きを合わせる強さと、最大の速さ |
| `APP_SWERVE_MAX_MOTOR_SPEED` | モーター 1 個の最大の速さ [rad/s] |

**初めて動かすとき**(タイヤを浮かせて)

1. まず `make TEST_MODE=1`(`APP` なし)で、2 個のモーターがそれぞれ正しい向きに回り、速度制御できることを確認します(上の「初めて動かすときの順番」)。
2. 次に `make APP=swerve TEST_MODE=1` を書き込みます。通信なしで、2 秒ごとに次の動きを繰り返します。
   1. 向きは前のまま、**タイヤが前に転がる** → 後ろに転がるなら `APP_SWERVE_WHEEL_RATIO` をマイナスに
   2. **上から見て反時計回りに 90° 回る**(左を向く)→ 逆に回るなら `APP_SWERVE_STEER_RATIO` をマイナスに
   3. 前に戻る
   4. 時計回りに 90° 回る(右を向く)
   - 90° より多く・少なく回るなら、`APP_SWERVE_STEER_RATIO` の大きさ(ギア比)を見直します。
3. 最後に、通常のビルド(`make APP=swerve NODE_ID=1` など)を書き込み、Pi で `swerve_drive` を起動して操縦します。

## CAN を実機で使う前に(クロックの精度)

今は STM32 の**内蔵クロック(HSI)**で動かしています。HSI の精度は常温で ±1% くらいですが、今の CAN の設定(1 Mbps、`main.c` の `MX_CAN_Init`)で許されるずれは**約 0.3%** しかありません。このままだと、**CAN の実機で通信エラーが出やすい**と考えられます(USB シリアルは許されるずれが大きいので、今の動作には影響しません)。

CAN を使う前に、どちらかにします。

- **外部の正確なクロック(HSE)を使う**:Nucleo-F303K8 は、ST-LINK の 8 MHz を STM32 の OSC_IN に入れられます。UM1956(Nucleo-32 のユーザーマニュアル)で、そのためのはんだブリッジがつながっているかを確認し、`SystemClock_Config` を HSE(バイパス)に変えます。自作基板なら水晶を載せます。
- **応急処置**:`MX_CAN_Init` の `TimeSeg1 = 11`、`TimeSeg2 = 4`、`SyncJumpWidth = 4` にすると、許されるずれが約 1% まで広がります(サンプル点 75%)。HSI のままでも動きやすくなりますが、温度が変わると外れることがあります。

## LED の意味

| 点滅 | 状態 |
|---|---|
| ゆっくり(1 秒周期) | Pi から指令を受け取って制御中 |
| 速い(0.2 秒周期) | 指令が届いていない(モーターは止まっている) |
| とても速い | エラー(初期化の失敗など)。CAN のトランシーバーや配線を確認 |

## 設定(`Core/Inc/app_config.h`)

| 設定 | 内容 |
|---|---|
| `APP_NODE_ID` | 基板の ID。`make NODE_ID=2` でも変えられる。Pi の `bridge.yaml` の `device_ids` と合わせる |
| `APP_COUNTS_PER_REV` | ホイール 1 回転あたりのカウント数 = エンコーダの PPR × 4 × 減速比 |
| `APP_SPEED_FILTER` | 速度のなめらかさ(0〜1)。速度は直近 8 ms のカウントから求め(`ENC_SPEED_WINDOW`)、さらにこれでなめらかにする |
| `APP_KFF` / `APP_KP` / `APP_KI` | 速度制御のゲイン |
| `APP_MOTOR1_DIR_SIGN` など | 回る向き・数える向きが逆なら -1 |

## 初めて動かすときの順番

1. **配線の確認**:ホイールを浮かせて `make TEST_MODE=1` を書き込みます。CAN なしで両方のモーターが前向きに 5 rad/s で回るはずです。
   - 逆に回る → `APP_MOTOR1_DIR_SIGN` を -1
   - すごい勢いで回り続ける、または止まりかけて震える → エンコーダの向きが逆なので `APP_MOTOR1_ENC_SIGN` を -1
2. **CAN の確認**:通常のビルドを書き込み、Pi で `stm_bridge` を起動します。LED が「速い点滅」から「ゆっくり点滅」に変われば、指令が届いています。`candump can0` で `101` と `201` が流れていることも確認できます。
3. **PID の調整**:`../stm32_can/README.md` の「動かし始めるときの順番」を見ながら、`APP_KFF` → `APP_KP` → `APP_KI` の順に調整します。

## CubeIDE で開きたい場合

デバッガ(ブレークポイントなど)を使いたいときは、CubeIDE の **File → Import → C/C++ → Existing Code as Makefile Project** で、このフォルダを読み込めます(ツールチェーンは「MCU ARM GCC」を選びます)。CubeMX の `.ioc` はないので、ピンやクロックを変えるときは `Core/Src/main.c` と `Core/Src/stm32f3xx_hal_msp.c` を直接書き換えます。
