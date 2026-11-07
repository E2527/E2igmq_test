# STM32 側のコード(CAN 通信 + モーターの速度制御)

Pi の `stm_bridge`(`ros2_ws/src/stm_bridge`)と CAN で通信し、エンコーダ付き DC モーター 2 個を速度制御します。bxCAN を持つ STM32(F303K8 など)の HAL 向けです。

F303K8 用にこれらを組み込んだ完成したプロジェクトが `../motor_node_f303k8/` にあります(`make` でビルドして、そのまま書き込めます)。自分で CubeIDE のプロジェクトを作って組み込む場合は、以下を参考にしてください。

Pi は**目標速度**を送り、**デューティ比は STM32 が決めます**。STM32 はエンコーダで実際の速度を測り、PID 制御でデューティ比を計算して PWM を出します。

```
Pi ──[目標速度 rad/s]──▶ STM32 ──PID──▶ デューティ比 ──▶ モータードライバー ──▶ モーター
   ◀─[実際の速度 rad/s]──       ◀───────────── エンコーダ ◀────────────────────┘
```

| ファイル | 内容 |
|---|---|
| `can_node.h` / `.c` | CAN の受信フィルタ、指令の受信(割り込み)、状態の定期送信 |
| `speed_pid.h` / `.c` | エンコーダのカウントから速度を求める計算と PID。HAL を使わない |
| `motor.h` / `.c` | エンコーダ用タイマー、PWM、方向ピンでモーター 1 個を速度制御する |
| `test/` | 上のコードを PC で動かすテスト(HAL の代わりの部品と、モーターの模型を使う) |

## 通信の決まり

| CAN ID(標準 11 ビット) | 向き | データ |
|---|---|---|
| `0x100 + ID` | Pi → STM32 | [モーター 1 の目標速度, モーター 2 の目標速度] [rad/s]。0 バイトは「止まる」 |
| `0x200 + ID` | STM32 → Pi | [モーター 1 の実際の速度, モーター 2 の実際の速度] [rad/s] |

- ID は基板ごとに 1〜255 で決めます。
- Pi は指令を 10 ms ごとに送り続けます。STM32 は 100 ms 以上届かなければモーターを止めます(Pi が落ちた、ケーブルが抜けた、など)。
- STM32 は状態を 10 ms ごとに送ります(`CAN_NODE_STATE_PERIOD_MS`)。
- 速度はホイール(減速後の出力軸)の回転速度 [rad/s] です。1 回転 = 2π rad ≒ 6.28 rad です。

## CubeMX の設定(Nucleo-F303K8 の例)

ピンは CubeMX が割り当てたものを、ボードのピン配置図で確認してください。

**CAN(1 Mbps)**
1. CAN を有効にし、CAN_RX / CAN_TX を `PA11` / `PA12` にします。
2. Parameter Settings(APB1 が 32 MHz のとき。Nucleo-F303K8 の既定のクロック設定)
   - Prescaler: 2、Bit Segment 1: 13 Times、Bit Segment 2: 2 Times、ReSynchronization Jump Width: 1 Time
   - → 1 Mbps(CubeMX の「Baud Rate」欄が 1000000 になることを確認)
   - Automatic Bus-Off Management: Enable、Automatic Retransmission: Enable
3. NVIC Settings で「USB low priority or CAN_RX0 interrupts」を有効にします。

**PWM(TIM1)**
- Channel1 / Channel2: PWM Generation CH1 / CH2
- タイマーのクロックが 64 MHz のとき、Prescaler = 0、Counter Period = 3199 → 20 kHz(モーターの音が聞こえにくい)

**エンコーダ(TIM2、TIM3)**
- Combined Channels: Encoder Mode、Encoder Mode: TI1 and TI2(4 逓倍)
- Counter Period: 65535(TIM2 も 65535 にする)
- エンコーダの信号にノイズが多いときは Input Filter を 4〜8 くらいにします。

**方向ピン**: モータードライバーの DIR(PHASE)ピンにつなぐ GPIO を 2 本、GPIO_Output にします(下の例では `MOTOR1_DIR`、`MOTOR2_DIR` と名前を付けたもの)。

## プロジェクトへの追加

`Core/Src` に `.c` を 3 つ、`Core/Inc` に `.h` を 3 つコピーします(`test/` はコピーしません)。

## main.c の例

```c
/* USER CODE BEGIN Includes */
#include "can_node.h"
#include "motor.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* 1 回転あたりのカウント数 = エンコーダの分解能(PPR)× 4 逓倍 × 減速比。例: 12 PPR、減速比 30 → 1440 */
#define COUNTS_PER_REV 1440.0f
static motor_t motors[2];
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
motors[0] = (motor_t){
    .pwm_tim = &htim1, .pwm_channel = TIM_CHANNEL_1,
    .dir_port = MOTOR1_DIR_GPIO_Port, .dir_pin = MOTOR1_DIR_Pin,
    .enc_tim = &htim2, .enc_sign = 1, .dir_sign = 1,
    .pid = {.kp = 0.02f, .ki = 0.5f, .kff = 1.0f / 30.0f, .max_duty = 0.95f},
};
motors[1] = (motor_t){
    .pwm_tim = &htim1, .pwm_channel = TIM_CHANNEL_2,
    .dir_port = MOTOR2_DIR_GPIO_Port, .dir_pin = MOTOR2_DIR_Pin,
    .enc_tim = &htim3, .enc_sign = 1, .dir_sign = 1,
    .pid = {.kp = 0.02f, .ki = 0.5f, .kff = 1.0f / 30.0f, .max_duty = 0.95f},
};
for (int i = 0; i < 2; i++) {
    enc_speed_init(&motors[i].speed, COUNTS_PER_REV, 0.2f);
    motor_start(&motors[i]);
}
if (can_node_init(&hcan, 1) != HAL_OK) {        /* この基板の ID。基板ごとに変える */
    Error_Handler();
}
uint32_t last_tick = HAL_GetTick();
/* USER CODE END 2 */

while (1) {
    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    if (now != last_tick) {                       /* 1 ms ごとに制御する */
        float dt = (float)(now - last_tick) / 1000.0f;
        last_tick = now;

        float target[2] = {0.0f, 0.0f};
        bool enabled = can_node_get_command(target, 2, 100) == 2;   /* 途切れたら止める */
        float speed[2];
        for (int i = 0; i < 2; i++) {
            speed[i] = motor_update(&motors[i], target[i], enabled, dt);
        }
        can_node_set_state(speed, 2);
    }
    can_node_update();                            /* 10 ms ごとに状態を Pi に送る */
    /* USER CODE END 3 */
}
```

他でも `HAL_CAN_RxFifo0MsgPendingCallback` を使っている場合は、`CAN_NODE_DEFINE_HAL_CALLBACKS` を 0 にして、自分のコールバックから `can_node_on_rx_fifo0()` を呼びます。

## 動かし始めるときの順番

1. **向きを合わせる**: `motor_set_duty(&motors[0], 0.2f)` で回し、ホイールが前に進む向きに回るか確かめます。逆なら `dir_sign = -1` にします。そのまま Pi で `stm1/state` を見て、速度がプラスになっているか確かめます。マイナスなら `enc_sign = -1` にします。
2. **kff を決める**: デューティ比 1.0 で回したときの速度 [rad/s] を測り(安全のため 0.5 で測って 2 倍してもよい)、`kff = 1 / その速度` にします。kff だけでも、だいたい狙った速さで回ります。
3. **kp を上げる**: 目標速度を変えたときに、ずれがすぐ縮まるまで kp を少しずつ上げます。ガタガタ振動し始めたら、少し下げます。
4. **ki で残りのずれを消す**: 負荷をかけても目標速度に戻るように、ki を上げます。行き過ぎて揺れるようなら下げます。

Pi(または PC)で `rqt_plot` を使い、`stm1/command/data[0]` と `stm1/state/data[0]` を重ねて表示すると調整しやすいです。

## PC でテストする

HAL の代わりの部品と、簡単なモーターの模型を使って、実機なしで動作を確かめられます。目標速度に届くか、負荷がかかっても戻るか、逆配線でも符号を合わせれば動くか、指令が途切れたら止まるか、などを確認します。

```bash
make -C firmware/stm32_can/test               # WSL(gcc が必要)
make -C /root/firmware/stm32_can/test         # dev コンテナの中
```

コードを変えたら実行してください。GitHub に push すると CI でも実行されます。

## 配線

- STM32 には CAN トランシーバーが入っていないので、3.3 V で動くトランシーバー(SN65HVD230 のモジュールなど)を付けます。STM32 の CAN_TX → トランシーバーの D(TX)、CAN_RX → R(RX)です。
- すべての基板の CANH 同士、CANL 同士をつなぎ、GND も共通にします。
- バスの両端(いちばん端の 2 か所)だけに 120 Ω の終端抵抗を入れます。トランシーバーのモジュールに 120 Ω が付いていることが多いので、端以外の基板では外します。
- 端から端までの抵抗(電源を切った状態で CANH-CANL 間)が約 60 Ω なら正しいです。
- エンコーダが 5 V 出力の場合は、5 V 耐性(FT)のピンか確認するか、分圧・レベル変換を入れます。
