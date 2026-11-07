#include "uart_node.h"

#include <string.h>

#define SYNC0 0xAAu
#define SYNC1 0x55u
#define MAX_DATA 8u
#define FRAME_MAX (2u + 3u + MAX_DATA + 2u)

enum { WAIT_SYNC0, WAIT_SYNC1, READ_BODY };

static UART_HandleTypeDef *s_huart;
static uint8_t s_id;
static uint8_t s_rx_byte;
static uint32_t s_last_send;

/* 受信中のフレーム: ID(2) + LEN + DATA + CRC(2) */
static uint8_t s_state_machine;
static uint8_t s_buf[3u + MAX_DATA + 2u];
static uint8_t s_idx;
static uint8_t s_need;

/* 割り込みとメインループの両方から触るので、メインループ側では割り込みを止めて読み書きする */
static float s_cmd[UART_NODE_MAX_FLOATS];
static uint8_t s_cmd_n;
static uint32_t s_cmd_tick;
static bool s_cmd_received;

static float s_state[UART_NODE_MAX_FLOATS];
static uint8_t s_state_n;
static uint8_t s_tx[FRAME_MAX];

static uint16_t crc16_modbus(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

static void start_receive(void)
{
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

static void handle_frame(uint16_t id, const uint8_t *data, uint8_t len)
{
    if (id != UART_NODE_COMMAND_BASE + s_id || len % 4u != 0u || len / 4u > UART_NODE_MAX_FLOATS) {
        return;
    }
    memcpy(s_cmd, data, len);
    s_cmd_n = (uint8_t)(len / 4u);   /* 0 なら「止まる」 */
    s_cmd_tick = HAL_GetTick();
    s_cmd_received = true;
}

static void feed(uint8_t byte)
{
    switch (s_state_machine) {
    case WAIT_SYNC0:
        if (byte == SYNC0) {
            s_state_machine = WAIT_SYNC1;
        }
        return;

    case WAIT_SYNC1:
        if (byte == SYNC1) {
            s_state_machine = READ_BODY;
            s_idx = 0;
            s_need = 3;                  /* まず ID(2)と LEN を読む */
        } else if (byte != SYNC0) {
            s_state_machine = WAIT_SYNC0;
        }
        return;

    default: /* READ_BODY */
        s_buf[s_idx++] = byte;
        if (s_idx == 3u) {
            if (s_buf[2] > MAX_DATA) {   /* 長さがおかしい = 偽の SYNC */
                s_state_machine = WAIT_SYNC0;
                return;
            }
            s_need = (uint8_t)(3u + s_buf[2] + 2u);
        }
        if (s_idx < s_need) {
            return;
        }
        s_state_machine = WAIT_SYNC0;
        uint8_t body_len = (uint8_t)(s_need - 2u);
        uint16_t rx_crc = (uint16_t)(s_buf[body_len] | (s_buf[body_len + 1u] << 8));
        if (crc16_modbus(s_buf, body_len) == rx_crc) {
            handle_frame((uint16_t)(s_buf[0] | (s_buf[1] << 8)), &s_buf[3], s_buf[2]);
        }
        return;
    }
}

HAL_StatusTypeDef uart_node_init(UART_HandleTypeDef *huart, uint8_t id)
{
    s_huart = huart;
    s_id = id;
    s_state_machine = WAIT_SYNC0;
    s_cmd_n = 0;
    s_cmd_received = false;
    s_state_n = 0;
    s_last_send = HAL_GetTick();
    return HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
}

void uart_node_set_state(const float *values, uint8_t n)
{
    if (n > UART_NODE_MAX_FLOATS) {
        n = UART_NODE_MAX_FLOATS;
    }
    __disable_irq();
    memcpy(s_state, values, (size_t)n * sizeof(float));
    s_state_n = n;
    __enable_irq();
}

uint8_t uart_node_get_command(float *values, uint8_t max, uint32_t timeout_ms)
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

void uart_node_update(void)
{
    uint32_t now = HAL_GetTick();
    if (now - s_last_send < UART_NODE_STATE_PERIOD_MS) {
        return;
    }
    if (s_huart->gState != HAL_UART_STATE_READY) {
        return;   /* 前の送信がまだ終わっていない */
    }
    s_last_send = now;

    uint16_t id = (uint16_t)(UART_NODE_STATE_BASE + s_id);
    s_tx[0] = SYNC0;
    s_tx[1] = SYNC1;
    s_tx[2] = (uint8_t)(id & 0xFFu);
    s_tx[3] = (uint8_t)(id >> 8);
    __disable_irq();
    uint8_t len = (uint8_t)(s_state_n * 4u);
    memcpy(&s_tx[5], s_state, len);
    __enable_irq();
    s_tx[4] = len;
    uint16_t crc = crc16_modbus(&s_tx[2], 3u + len);
    s_tx[5u + len] = (uint8_t)(crc & 0xFFu);
    s_tx[6u + len] = (uint8_t)(crc >> 8);
    HAL_UART_Transmit_IT(s_huart, s_tx, (uint16_t)(7u + len));
}

void uart_node_on_rx_cplt(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) {
        return;
    }
    feed(s_rx_byte);
    start_receive();
}

void uart_node_on_error(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) {
        return;
    }
    /* オーバーランやノイズで受信が止まるので、読み直す */
    s_state_machine = WAIT_SYNC0;
    start_receive();
}

#if UART_NODE_DEFINE_HAL_CALLBACKS
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_node_on_rx_cplt(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uart_node_on_error(huart);
}
#endif
