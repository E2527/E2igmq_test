# STM32 側のシリアルバス通信

Pi の `stm_bridge`(`ros2_ws/src/stm_bridge`)と、RS-485 などの1本のバスで通信するためのコードです。HAL を使う STM32(F7 / H7 / G4 など)向けです。

| ファイル | 内容 |
|---|---|
| `bus_protocol.h` / `.c` | パケットの組み立てと読み取り、CRC。HAL に依存しない |
| `bus_slave.h` / `.c` | 子機としての受信と返信(HAL の UART 割り込みを使う) |

## 通信の流れ

Pi が親機になり、決まった周期で ID ごとに順番に問い合わせます。STM32 は自分の ID 宛ての問い合わせにだけ返信します。

```
Pi  → [ID=1 指令値]   STM32 #1 → [状態]
Pi  → [ID=2 指令値]   STM32 #2 → [状態]
...(rate_hz の周期で繰り返す)
```

パケットの形式:

```
0xAA 0x55 | ID | CMD | LEN | PAYLOAD | CRC16(下位, 上位)
```

- 指令値と状態は、どちらも float の並び(最大 16 個)です。意味(何番目が何の値か)はチームで決めます。
- 空の指令(個数 0)や、指令が届かなくなったときは「止まる」として扱います。Pi 側が止まってもロボットが暴走しないようにするためです。

## プロジェクトへの追加

1. 4 つのファイルを CubeIDE のプロジェクトの `Core/Src` と `Core/Inc` にコピーします。
2. CubeMX で、バスに使う USART を設定します。
   - Mode: Asynchronous
   - Baud Rate: Pi の `bridge.yaml` の `baudrate` と同じ(既定 1000000)
   - NVIC: USART global interrupt を有効にする
   - Hardware Flow Control (RS485): Enable(DE ピンを自動で制御)
3. `main.c` で初期化し、メインループで指令を読み、状態を書きます。

```c
/* USER CODE BEGIN Includes */
#include "bus_slave.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
bus_slave_init(&huart1, 1);                     /* この基板の ID。基板ごとに変える */
/* USER CODE END 2 */

while (1) {
    float cmd[4];
    uint8_t n = bus_slave_get_command(cmd, 4, 100);
    if (n == 0) {
        /* 指令なし、または 100 ms 以上届いていない → モーターを止める */
    } else {
        /* cmd[0] 〜 cmd[n-1] を目標値として使う */
    }
    float state[2] = { /* 速度 */ 0.0f, /* 電流 */ 0.0f };
    bus_slave_set_state(state, 2);
}
```

- DE ピンを GPIO で制御する場合は、`BUS_SOFT_DE` を 1 にし、`BUS_DE_GPIO_Port` / `BUS_DE_Pin` を定義します(コンパイラの定義か、`bus_slave.h` を読み込む前)。
- 他の UART でも `HAL_UART_RxCpltCallback` などを使っている場合は、`BUS_SLAVE_DEFINE_HAL_CALLBACKS` を 0 にして、自分のコールバックから `bus_slave_on_rx_cplt()` / `bus_slave_on_tx_cplt()` / `bus_slave_on_error()` を呼びます。

## 配線(RS-485)

- すべての基板の A 同士、B 同士をつなぎ、GND も共通にします。
- バスの両端(いちばん端の 2 か所)だけに 120 Ω の終端抵抗を入れます。
- Pi 側は USB-RS485 変換アダプタを使います。1 Mbps で使うなら、FTDI(FT232R など)や CH343 のアダプタを選んでください。
