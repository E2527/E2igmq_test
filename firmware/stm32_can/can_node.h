/*
 * CAN バスの 1 台として、Pi の stm_bridge と通信する(STM32 bxCAN / HAL 用。F303K8 など)
 * Pi 側の ros2_ws/src/stm_bridge/stm_bridge/protocol.py と同じ決まり:
 *
 *   0x100 + ID … Pi → STM32 の指令値(標準 ID、float 2 個まで。0 バイトは「止まる」)
 *   0x200 + ID … STM32 → Pi の状態(float 2 個まで)
 *
 * 使い方(main.c):
 *
 *   #include "can_node.h"
 *
 *   can_node_init(&hcan, 1);                       // CAN と、この基板の ID(1〜255)
 *   while (1) {
 *       float cmd[2];
 *       uint8_t n = can_node_get_command(cmd, 2, 100);
 *       if (n == 0) {
 *           // 指令がない / 100 ms 以上届いていない → 止める
 *       } else {
 *           // cmd[0]〜cmd[n-1] を目標値として使う
 *       }
 *       float state[2] = {speed, current};
 *       can_node_set_state(state, 2);
 *       can_node_update();                         // CAN_NODE_STATE_PERIOD_MS ごとに状態を送る
 *   }
 *
 * このファイルは HAL_CAN_RxFifo0MsgPendingCallback を定義する。
 * 他でもこのコールバックを使っている場合は CAN_NODE_DEFINE_HAL_CALLBACKS を 0 にし、
 * 自分のコールバックから can_node_on_rx_fifo0() を呼ぶ。
 */
#ifndef CAN_NODE_H
#define CAN_NODE_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#define CAN_NODE_COMMAND_BASE 0x100u
#define CAN_NODE_STATE_BASE 0x200u
#define CAN_NODE_MAX_FLOATS 2

#ifndef CAN_NODE_STATE_PERIOD_MS
#define CAN_NODE_STATE_PERIOD_MS 10u
#endif

#ifndef CAN_NODE_DEFINE_HAL_CALLBACKS
#define CAN_NODE_DEFINE_HAL_CALLBACKS 1
#endif

/* 受信フィルタを自分の指令 ID だけに設定し、CAN を開始する。失敗したら HAL_ERROR などを返す */
HAL_StatusTypeDef can_node_init(CAN_HandleTypeDef *hcan, uint8_t id);

/* Pi に送る状態を更新する(n は CAN_NODE_MAX_FLOATS まで) */
void can_node_set_state(const float *values, uint8_t n);

/* 最新の指令を values に入れて個数を返す。0 は「止まる」(空の指令、または timeout_ms 以上届いていない) */
uint8_t can_node_get_command(float *values, uint8_t max, uint32_t timeout_ms);

/* メインループで呼ぶ。CAN_NODE_STATE_PERIOD_MS ごとに状態を送る */
void can_node_update(void);

void can_node_on_rx_fifo0(CAN_HandleTypeDef *hcan);

#endif /* CAN_NODE_H */
