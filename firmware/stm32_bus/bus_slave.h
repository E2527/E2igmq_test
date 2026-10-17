/*
 * シリアルバス(RS-485 など)の子機として、Pi の stm_bridge と通信する(STM32 HAL 用)
 *
 * 受信は 1 バイトずつの割り込みで行い、自分の ID 宛ての指令が届いたら、割り込みの中ですぐに
 * 最新の状態を返信する。メインループが重い処理をしていても返信が遅れない。
 *
 * 使い方(main.c):
 *
 *   #include "bus_slave.h"
 *
 *   bus_slave_init(&huart1, 1);                   // UART と自分の ID(1〜247)
 *   while (1) {
 *       float cmd[4];
 *       uint8_t n = bus_slave_get_command(cmd, 4, 100);
 *       if (n == 0) {
 *           // 指令がない / 100 ms 以上届いていない → 止める
 *       } else {
 *           // cmd[0]〜cmd[n-1] を目標値として使う
 *       }
 *       float state[2] = {speed, current};
 *       bus_slave_set_state(state, 2);             // 次の返信で Pi に送られる
 *   }
 *
 * CubeMX の設定:
 *   - バスに使う USART を Asynchronous、Pi の stm_bridge の baudrate と同じ速さにし、割り込みを有効にする
 *   - RS-485 トランシーバの DE ピンは、USART の「Hardware Flow Control (RS485)」で自動制御するのがおすすめ
 *     (F7 / H7 / G4 は対応)。GPIO で制御する場合は BUS_SOFT_DE を 1 にして、ピンを下で指定する
 *
 * このファイルは HAL_UART_RxCpltCallback / TxCpltCallback / ErrorCallback を定義する。
 * 他の UART でもコールバックを使っている場合は BUS_SLAVE_DEFINE_HAL_CALLBACKS を 0 にし、
 * 自分のコールバックから bus_slave_on_rx_cplt() などを呼ぶ。
 */
#ifndef BUS_SLAVE_H
#define BUS_SLAVE_H

#include "main.h"
#include "bus_protocol.h"

#ifndef BUS_SLAVE_DEFINE_HAL_CALLBACKS
#define BUS_SLAVE_DEFINE_HAL_CALLBACKS 1
#endif

#ifndef BUS_SOFT_DE
#define BUS_SOFT_DE 0
#endif
#if BUS_SOFT_DE
#ifndef BUS_DE_GPIO_Port
#define BUS_DE_GPIO_Port GPIOA
#define BUS_DE_Pin GPIO_PIN_8
#endif
#endif

#define BUS_MAX_FLOATS (BUS_MAX_PAYLOAD / 4)

void bus_slave_init(UART_HandleTypeDef *huart, uint8_t id);

/* Pi に返す状態を更新する(n は BUS_MAX_FLOATS まで) */
void bus_slave_set_state(const float *values, uint8_t n);

/* 最新の指令を values に入れて個数を返す。0 は「止まる」(空の指令、または timeout_ms 以上届いていない) */
uint8_t bus_slave_get_command(float *values, uint8_t max, uint32_t timeout_ms);

void bus_slave_on_rx_cplt(UART_HandleTypeDef *huart);
void bus_slave_on_tx_cplt(UART_HandleTypeDef *huart);
void bus_slave_on_error(UART_HandleTypeDef *huart);

#endif /* BUS_SLAVE_H */
