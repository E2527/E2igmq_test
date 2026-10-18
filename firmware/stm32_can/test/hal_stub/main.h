/*
 * PC でテストするための STM32 HAL の代わり(本物の main.h / HAL の代わりに読み込まれる)
 * 名前と値は stm32f3xx_hal の CAN / TIM / GPIO に合わせてある。中身はテスト用の変数を読み書きするだけ。
 */
#ifndef TEST_HAL_STUB_MAIN_H
#define TEST_HAL_STUB_MAIN_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
#define ENABLE 1u
#define DISABLE 0u

/* ---- GPIO ---- */
typedef struct { int pin_state[16]; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
#define GPIO_PIN_0 ((uint16_t)0x0001)
#define GPIO_PIN_1 ((uint16_t)0x0002)
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);

/* ---- TIM ---- */
typedef struct {
    uint32_t cnt;
    uint32_t arr;
    uint32_t ccr[4];
    int pwm_started;
    int enc_started;
} TIM_HandleTypeDef;
#define TIM_CHANNEL_1 0x00000000u
#define TIM_CHANNEL_2 0x00000004u
#define TIM_CHANNEL_ALL 0x0000003Cu
#define __HAL_TIM_GET_COUNTER(h) ((h)->cnt)
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->arr)
#define __HAL_TIM_SET_COMPARE(h, ch, v) ((h)->ccr[(ch) / 4u] = (v))
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *h, uint32_t channel);

/* ---- CAN(bxCAN) ---- */
#define CAN_FILTERMODE_IDMASK 0u
#define CAN_FILTERSCALE_32BIT 1u
#define CAN_RX_FIFO0 0u
#define CAN_ID_STD 0u
#define CAN_ID_EXT 4u
#define CAN_RTR_DATA 0u
#define CAN_IT_RX_FIFO0_MSG_PENDING 2u
typedef struct {
    uint32_t FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow;
    uint32_t FilterFIFOAssignment, FilterBank, FilterMode, FilterScale, FilterActivation;
    uint32_t SlaveStartFilterBank;
} CAN_FilterTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, Timestamp, FilterMatchIndex; } CAN_RxHeaderTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef struct { int dummy; } CAN_HandleTypeDef;
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, const CAN_FilterTypeDef *f);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t it);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, const CAN_TxHeaderTypeDef *header,
                                       const uint8_t data[], uint32_t *mailbox);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t fifo);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                       CAN_RxHeaderTypeDef *header, uint8_t data[]);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h);

/* ---- その他 ---- */
uint32_t HAL_GetTick(void);
void __disable_irq(void);
void __enable_irq(void);

/* ---- テストから操作する変数(hal_stub.c) ---- */
extern uint32_t stub_tick;
extern CAN_FilterTypeDef stub_can_filter;
extern uint32_t stub_can_tx_free;
extern CAN_TxHeaderTypeDef stub_can_tx_header;
extern uint8_t stub_can_tx_data[8];
extern int stub_can_tx_count;
void stub_can_receive(CAN_HandleTypeDef *h, uint32_t std_id, uint32_t ide, const void *data, uint32_t dlc);

#endif /* TEST_HAL_STUB_MAIN_H */
