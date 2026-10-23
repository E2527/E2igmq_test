/*
 * 通信の線(CAN / USB シリアル)を make のときに選ぶ
 *   make → CAN(can_node.c)、make LINK=uart → USB シリアル(uart_node.c)
 */
#ifndef LINK_H
#define LINK_H

#include "app_config.h"
#include "can_node.h"
#include "main.h"
#include "uart_node.h"

#if APP_LINK_UART
#define link_init() uart_node_init(&huart2, APP_NODE_ID)
#define link_get_command uart_node_get_command
#define link_set_state uart_node_set_state
#define link_update uart_node_update
#else
#define link_init() can_node_init(&hcan, APP_NODE_ID)
#define link_get_command can_node_get_command
#define link_set_state can_node_set_state
#define link_update can_node_update
#endif

#endif /* LINK_H */
