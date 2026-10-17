#include "can_node.h"

#include <string.h>

static CAN_HandleTypeDef *s_hcan;
static uint8_t s_id;
static uint32_t s_last_send;

/* 割り込みとメインループの両方から触るので、メインループ側では割り込みを止めて読み書きする */
static float s_cmd[CAN_NODE_MAX_FLOATS];
static uint8_t s_cmd_n;
static uint32_t s_cmd_tick;
static bool s_cmd_received;

static float s_state[CAN_NODE_MAX_FLOATS];
static uint8_t s_state_n;

HAL_StatusTypeDef can_node_init(CAN_HandleTypeDef *hcan, uint8_t id)
{
    s_hcan = hcan;
    s_id = id;
    s_cmd_n = 0;
    s_cmd_received = false;
    s_state_n = 0;
    s_last_send = HAL_GetTick();

    /* 32 ビットのマスクモードで、標準 ID = 0x100 + id のデータフレームだけを FIFO0 に入れる。
     * レジスタの並び: 上位 16 ビットの bit15〜5 が標準 ID、下位 16 ビットの bit2 が IDE、bit1 が RTR */
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (uint32_t)((CAN_NODE_COMMAND_BASE + id) << 5);
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = (uint32_t)(0x7FFu << 5);
    filter.FilterMaskIdLow = 0x0006;               /* IDE = 0(標準)、RTR = 0(データ)だけ通す */
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    HAL_StatusTypeDef st = HAL_CAN_ConfigFilter(hcan, &filter);
    if (st == HAL_OK) {
        st = HAL_CAN_Start(hcan);
    }
    if (st == HAL_OK) {
        st = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    return st;
}

void can_node_set_state(const float *values, uint8_t n)
{
    if (n > CAN_NODE_MAX_FLOATS) {
        n = CAN_NODE_MAX_FLOATS;
    }
    __disable_irq();
    memcpy(s_state, values, (size_t)n * sizeof(float));
    s_state_n = n;
    __enable_irq();
}

uint8_t can_node_get_command(float *values, uint8_t max, uint32_t timeout_ms)
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

void can_node_update(void)
{
    uint32_t now = HAL_GetTick();
    if (now - s_last_send < CAN_NODE_STATE_PERIOD_MS) {
        return;
    }
    s_last_send = now;

    /* 送信用のメールボックスが空いていなければ今回は送らない(バスに誰もいないときなど) */
    if (HAL_CAN_GetTxMailboxesFreeLevel(s_hcan) == 0) {
        return;
    }
    uint8_t data[8];
    CAN_TxHeaderTypeDef header = {0};
    header.StdId = CAN_NODE_STATE_BASE + s_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.TransmitGlobalTime = DISABLE;
    __disable_irq();
    header.DLC = (uint32_t)s_state_n * 4u;
    memcpy(data, s_state, header.DLC);
    __enable_irq();
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(s_hcan, &header, data, &mailbox);
}

void can_node_on_rx_fifo0(CAN_HandleTypeDef *hcan)
{
    if (hcan != s_hcan) {
        return;
    }
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
            return;
        }
        if (header.IDE != CAN_ID_STD || header.StdId != CAN_NODE_COMMAND_BASE + s_id
            || header.DLC % 4 != 0 || header.DLC / 4 > CAN_NODE_MAX_FLOATS) {
            continue;
        }
        memcpy(s_cmd, data, header.DLC);
        s_cmd_n = (uint8_t)(header.DLC / 4);   /* 0 なら「止まる」 */
        s_cmd_tick = HAL_GetTick();
        s_cmd_received = true;
    }
}

#if CAN_NODE_DEFINE_HAL_CALLBACKS
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_node_on_rx_fifo0(hcan);
}
#endif
