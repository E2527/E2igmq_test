#include "bus_slave.h"

#include <string.h>

static UART_HandleTypeDef *s_huart;
static uint8_t s_id;
static uint8_t s_rx_byte;
static bus_parser_t s_parser;
static bus_frame_t s_frame;
static uint8_t s_tx[BUS_MAX_FRAME];

/* 割り込みとメインループの両方から触るので、メインループ側では割り込みを止めて読み書きする */
static float s_state[BUS_MAX_FLOATS];
static uint8_t s_state_n;
static float s_cmd[BUS_MAX_FLOATS];
static uint8_t s_cmd_n;
static uint32_t s_cmd_tick;
static bool s_cmd_received;

static void start_receive(void)
{
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

static void handle_frame(void)
{
    /* 他の ID 宛て、他の子機の返信、自分の返信のエコーは無視する */
    if (s_frame.id != s_id || (s_frame.cmd & BUS_REPLY_FLAG)) {
        return;
    }
    if (s_huart->gState != HAL_UART_STATE_READY) {
        return; /* 前の返信をまだ送っている */
    }

    size_t n;
    if (s_frame.cmd == BUS_CMD_DATA) {
        if (s_frame.len % 4 != 0) {
            return;
        }
        memcpy(s_cmd, s_frame.payload, s_frame.len);
        s_cmd_n = (uint8_t)(s_frame.len / 4);
        s_cmd_tick = HAL_GetTick();
        s_cmd_received = true;
        n = bus_encode(s_id, BUS_CMD_DATA | BUS_REPLY_FLAG, (const uint8_t *)s_state,
                       (uint8_t)(s_state_n * 4), s_tx);
    } else if (s_frame.cmd == BUS_CMD_PING) {
        n = bus_encode(s_id, BUS_CMD_PING | BUS_REPLY_FLAG, NULL, 0, s_tx);
    } else {
        return;
    }

#if BUS_SOFT_DE
    HAL_GPIO_WritePin(BUS_DE_GPIO_Port, BUS_DE_Pin, GPIO_PIN_SET);
#endif
    HAL_UART_Transmit_IT(s_huart, s_tx, (uint16_t)n);
}

void bus_slave_init(UART_HandleTypeDef *huart, uint8_t id)
{
    s_huart = huart;
    s_id = id;
    s_state_n = 0;
    s_cmd_n = 0;
    s_cmd_received = false;
    bus_parser_init(&s_parser);
#if BUS_SOFT_DE
    HAL_GPIO_WritePin(BUS_DE_GPIO_Port, BUS_DE_Pin, GPIO_PIN_RESET);
#endif
    start_receive();
}

void bus_slave_set_state(const float *values, uint8_t n)
{
    if (n > BUS_MAX_FLOATS) {
        n = BUS_MAX_FLOATS;
    }
    __disable_irq();
    memcpy(s_state, values, (size_t)n * sizeof(float));
    s_state_n = n;
    __enable_irq();
}

uint8_t bus_slave_get_command(float *values, uint8_t max, uint32_t timeout_ms)
{
    __disable_irq();
    uint8_t n = s_cmd_n;
    bool fresh = s_cmd_received && (HAL_GetTick() - s_cmd_tick) < timeout_ms;
    if (n > max) {
        n = max;
    }
    memcpy(values, s_cmd, (size_t)n * sizeof(float));
    __enable_irq();
    return fresh ? n : 0;
}

void bus_slave_on_rx_cplt(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) {
        return;
    }
    if (bus_parser_feed(&s_parser, s_rx_byte, &s_frame)) {
        handle_frame();
    }
    start_receive();
}

void bus_slave_on_tx_cplt(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) {
        return;
    }
#if BUS_SOFT_DE
    /* 最後のビットまで送り終わってから DE を下げる(TxCplt は送信完了 = TC で呼ばれる) */
    HAL_GPIO_WritePin(BUS_DE_GPIO_Port, BUS_DE_Pin, GPIO_PIN_RESET);
#endif
}

void bus_slave_on_error(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) {
        return;
    }
    /* オーバーランやノイズで受信が止まるので、読み直す */
    bus_parser_init(&s_parser);
    start_receive();
}

#if BUS_SLAVE_DEFINE_HAL_CALLBACKS
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { bus_slave_on_rx_cplt(huart); }
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) { bus_slave_on_tx_cplt(huart); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) { bus_slave_on_error(huart); }
#endif
