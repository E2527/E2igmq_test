# STM32 側の CAN 通信

Pi の `stm_bridge`(`ros2_ws/src/stm_bridge`)と CAN で通信するためのコードです。bxCAN を持つ STM32(F303K8 など)の HAL 向けです。

| ファイル | 内容 |
|---|---|
| `can_node.h` / `.c` | 受信フィルタの設定、指令の受信(割り込み)、状態の定期送信 |

## 通信の決まり

| CAN ID(標準 11 ビット) | 向き | データ |
|---|---|---|
| `0x100 + ID` | Pi → STM32 | 指令値(float 2 個まで)。0 バイトは「止まる」 |
| `0x200 + ID` | STM32 → Pi | 状態(float 2 個まで) |

- ID は基板ごとに 1〜255 で決めます。
- Pi は指令を 10 ms ごとに送り続けます。STM32 は 100 ms 以上届かなければ止まります(Pi が落ちた、ケーブルが抜けた、など)。
- STM32 は状態を 10 ms ごとに送ります(`CAN_NODE_STATE_PERIOD_MS`)。
- 何番目の値が何か(速度、電流など)はチームで決めます。

## CubeMX の設定(Nucleo-F303K8、1 Mbps の場合)

1. **CAN** を有効にし、ピンを `PA11`(CAN_RX)/ `PA12`(CAN_TX)にします(Nucleo-32 の D10 / D2。ボードの図で確認してください)。
2. **Parameter Settings**(APB1 が 32 MHz のとき。Nucleo-F303K8 の既定のクロック設定)
   - Prescaler: 2
   - Time Quanta in Bit Segment 1: 13 Times
   - Time Quanta in Bit Segment 2: 2 Times
   - ReSynchronization Jump Width: 1 Time
   - → 1 Mbps、サンプル点 87.5 %(CubeMX の「Baud Rate」欄が 1000000 になることを確認)
   - Automatic Bus-Off Management: Enable(エラーで止まっても自動で復帰する)
   - Automatic Retransmission: Enable
3. **NVIC Settings** で「USB low priority or CAN_RX0 interrupts」を有効にします。
4. 生成したプロジェクトの `Core/Src` に `can_node.c`、`Core/Inc` に `can_node.h` をコピーします。

APB1 の周波数が違う場合や 500 kbps にしたい場合は、CubeMX の「Baud Rate」欄を見ながら Prescaler と Bit Segment を合わせ、Pi 側(`docker/pi/80-can.network` の `BitRate`)も同じにします。

## main.c

```c
/* USER CODE BEGIN Includes */
#include "can_node.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
if (can_node_init(&hcan, 1) != HAL_OK) {        /* この基板の ID。基板ごとに変える */
    Error_Handler();
}
/* USER CODE END 2 */

while (1) {
    float cmd[2];
    uint8_t n = can_node_get_command(cmd, 2, 100);
    if (n == 0) {
        /* 指令なし、または 100 ms 以上届いていない → モーターを止める */
    } else {
        /* cmd[0] 〜 cmd[n-1] を目標値として使う */
    }
    float state[2] = { /* 速度 */ 0.0f, /* 電流 */ 0.0f };
    can_node_set_state(state, 2);
    can_node_update();
}
```

他でも `HAL_CAN_RxFifo0MsgPendingCallback` を使っている場合は、`CAN_NODE_DEFINE_HAL_CALLBACKS` を 0 にして、自分のコールバックから `can_node_on_rx_fifo0()` を呼びます。

## 配線

- STM32 には CAN トランシーバーが入っていないので、3.3 V で動くトランシーバー(SN65HVD230 のモジュールなど)を付けます。STM32 の CAN_TX → トランシーバーの D(TX)、CAN_RX → R(RX)です。
- すべての基板の CANH 同士、CANL 同士をつなぎ、GND も共通にします。
- バスの両端(いちばん端の 2 か所)だけに 120 Ω の終端抵抗を入れます。トランシーバーのモジュールに 120 Ω が付いていることが多いので、端以外の基板では外します。
- 端から端までの抵抗(電源を切った状態で CANH-CANL 間)が約 60 Ω なら正しいです。
