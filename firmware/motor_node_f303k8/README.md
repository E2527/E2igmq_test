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
```

最後に Flash と RAM の使用量が表示されます(Flash 約 11 KB / 64 KB、RAM 約 2 KB / 12 KB)。

## 書き込み

Windows の **STM32CubeProgrammer** を使います(CubeIDE を入れていれば一緒に入っています)。

1. Nucleo を USB で PC につなぎます。
2. CubeProgrammer の右上で「ST-LINK」を選び、**Connect** を押します。
3. 左の「Erasing & programming」で、ファイルに次を指定します(エクスプローラーのアドレス欄に貼り付けて開けます)。
   ```
   \\wsl$\Ubuntu\home\souta\e2igmq_test\firmware\motor_node_f303k8\build\node1\motor_node.elf
   ```
4. **Start Programming** を押します。終わったら Disconnect して、Nucleo のリセットボタンを押します。

## USB シリアルで PC とつなぐ(CAN の部品がないとき)

Nucleo の USB には ST-LINK が入っていて、書き込みだけでなく USB シリアル(仮想 COM ポート)にもなります。STM32 の USART2(PA2 / PA15)がこれにつながっているので、**USB ケーブル 1 本で**、CAN と同じ中身の通信ができます。

```
PC(WSL の dev コンテナ)── USB ──▶ ST-LINK(/dev/ttyACM0)── USART2 ──▶ STM32
```

1. **USB シリアル版を書き込む**:`make LINK=uart -j4` でビルドし、`build/node1-uart/motor_node.elf` を CubeProgrammer で書き込みます。LD3 が速く点滅(指令待ち)すれば OK です。
2. **Windows の USB を WSL に渡す**(usbipd-win。Windows の PowerShell で)
   ```powershell
   winget install usbipd                # 初回だけ。インストール後に PowerShell を開き直す
   usbipd list                          # 「ST-Link」の行の BUSID(例 2-3)を確認
   usbipd bind --busid 2-3              # 初回だけ。管理者の PowerShell で
   usbipd attach --wsl --busid 2-3      # 挿し直すたびに実行
   ```
   WSL で `ls /dev/ttyACM*` を実行して `/dev/ttyACM0` が出れば OK です。
3. **dev コンテナでブリッジを起動する**(dev コンテナは `/dev` を共有しているので、そのまま見えます)
   ```bash
   ros2 run stm_bridge bridge --ros-args -p can_interface:=serial:/dev/ttyACM0
   ```
   別のターミナルで指令を送ると、LD3 が「ゆっくり点滅」に変わり、`stm1/state` に速度が返ってきます。
   ```bash
   ros2 topic pub -r 20 stm1/command std_msgs/msg/Float32MultiArray "{data: [1.0, 1.0]}"
   ros2 topic echo stm1/state
   ```

**注意**:USB を WSL に渡している間は、Windows の CubeProgrammer から ST-LINK が見えません。書き込むときは、先に PowerShell で `usbipd detach --busid 2-3` を実行します。

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
