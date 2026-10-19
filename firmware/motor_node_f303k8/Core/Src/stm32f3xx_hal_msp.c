/*
 * 周辺回路ごとのクロック・ピン・割り込みの設定(CubeMX の stm32f3xx_hal_msp.c と同じ役割)
 */
#include "main.h"

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan_)
{
    if (hcan_->Instance != CAN) {
        return;
    }
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA11 = CAN_RX、PA12 = CAN_TX */
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF9_CAN;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_NVIC_SetPriority(CAN_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN_SCE_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN_SCE_IRQn);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;                        /* オープンコレクタ出力のエンコーダにも対応 */
    g.Speed = GPIO_SPEED_FREQ_LOW;

    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        g.Pin = GPIO_PIN_0 | GPIO_PIN_1;         /* PA0 = TIM2_CH1、PA1 = TIM2_CH2 */
        g.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &g);
    } else if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        g.Pin = GPIO_PIN_4 | GPIO_PIN_5;         /* PB4 = TIM3_CH1、PB5 = TIM3_CH2 */
        g.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOB, &g);
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) {
        return;
    }
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA8 = TIM1_CH1、PA9 = TIM1_CH2 */
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &g);
}
