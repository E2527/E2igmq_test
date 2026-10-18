#include <string.h>

#include "main.h"

uint32_t stub_tick;
CAN_FilterTypeDef stub_can_filter;
uint32_t stub_can_tx_free = 3;
CAN_TxHeaderTypeDef stub_can_tx_header;
uint8_t stub_can_tx_data[8];
int stub_can_tx_count;

static CAN_RxHeaderTypeDef s_rx_header;
static uint8_t s_rx_data[8];
static uint32_t s_rx_pending;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    for (int i = 0; i < 16; i++) {
        if (pin & (1u << i)) {
            port->pin_state[i] = (int)state;
        }
    }
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel)
{
    (void)channel;
    h->pwm_started = 1;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *h, uint32_t channel)
{
    (void)channel;
    h->enc_started = 1;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, const CAN_FilterTypeDef *f)
{
    (void)h;
    stub_can_filter = *f;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h)
{
    (void)h;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t it)
{
    (void)h;
    (void)it;
    return HAL_OK;
}

uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h)
{
    (void)h;
    return stub_can_tx_free;
}

HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, const CAN_TxHeaderTypeDef *header,
                                       const uint8_t data[], uint32_t *mailbox)
{
    (void)h;
    stub_can_tx_header = *header;
    memcpy(stub_can_tx_data, data, header->DLC);
    *mailbox = 1;
    stub_can_tx_count++;
    return HAL_OK;
}

uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t fifo)
{
    (void)h;
    (void)fifo;
    return s_rx_pending;
}

HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                       CAN_RxHeaderTypeDef *header, uint8_t data[])
{
    (void)h;
    (void)fifo;
    *header = s_rx_header;
    memcpy(data, s_rx_data, 8);
    s_rx_pending = 0;
    return HAL_OK;
}

void stub_can_receive(CAN_HandleTypeDef *h, uint32_t std_id, uint32_t ide, const void *data, uint32_t dlc)
{
    memset(&s_rx_header, 0, sizeof(s_rx_header));
    s_rx_header.StdId = std_id;
    s_rx_header.IDE = ide;
    s_rx_header.DLC = dlc;
    memset(s_rx_data, 0, sizeof(s_rx_data));
    memcpy(s_rx_data, data, dlc);
    s_rx_pending = 1;
    HAL_CAN_RxFifo0MsgPendingCallback(h);
}

uint32_t HAL_GetTick(void)
{
    return stub_tick;
}

void __disable_irq(void)
{
}

void __enable_irq(void)
{
}
