/*
 * USB シリアル(UART)で Pi / PC の stm_bridge と通信する(can_node の代わりに使える)
 *
 * CAN と同じ中身(ID と 8 バイトまでのデータ)を、区切りと CRC を付けて UART で送る。
 * Nucleo なら USART2(PA2 / PA15)が ST-LINK の仮想 COM ポートにつながっているので、
 * USB ケーブル 1 本で Pi / PC と通信できる。
 *
 *   0xAA 0x55 | ID(下位, 上位) | LEN | DATA(LEN バイト、8 まで) | CRC16(下位, 上位)
 *
 *   0x100 + ID … Pi → STM32 の指令値(float 2 個まで。0 バイトは「止まる」)
 *   0x200 + ID … STM32 → Pi の状態(float 2 個まで)
 *
 * Pi / PC 側は stm_bridge の can_interface を serial:/dev/ttyACM0 にする。
 *
 * このファイルは HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback を定義する。
 * 他の UART でもコールバックを使っている場合は UART_NODE_DEFINE_HAL_CALLBACKS を 0 にし、
 * 自分のコールバックから uart_node_on_rx_cplt() / uart_node_on_error() を呼ぶ。
 */
#ifndef UART_NODE_H
#define UART_NODE_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#define UART_NODE_COMMAND_BASE 0x100u
#define UART_NODE_STATE_BASE 0x200u
#define UART_NODE_MAX_FLOATS 2

#ifndef UART_NODE_STATE_PERIOD_MS
#define UART_NODE_STATE_PERIOD_MS 10u
#endif

#ifndef UART_NODE_DEFINE_HAL_CALLBACKS
#define UART_NODE_DEFINE_HAL_CALLBACKS 1
#endif

/* 受信を開始する。失敗したら HAL_ERROR などを返す */
HAL_StatusTypeDef uart_node_init(UART_HandleTypeDef *huart, uint8_t id);

/* Pi に送る状態を更新する(n は UART_NODE_MAX_FLOATS まで) */
void uart_node_set_state(const float *values, uint8_t n);

/* 最新の指令を values に入れて個数を返す。0 は「止まる」(空の指令、または timeout_ms 以上届いていない) */
uint8_t uart_node_get_command(float *values, uint8_t max, uint32_t timeout_ms);

/* メインループで呼ぶ。UART_NODE_STATE_PERIOD_MS ごとに状態を送る */
void uart_node_update(void);

void uart_node_on_rx_cplt(UART_HandleTypeDef *huart);
void uart_node_on_error(UART_HandleTypeDef *huart);

#endif /* UART_NODE_H */
