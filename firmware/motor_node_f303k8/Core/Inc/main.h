/*
 * STM32F303K8 モーター制御ノード
 * ピンの割り当て(Nucleo-F303K8 の表記は UM1956 のピン配置図で確認してください)
 *
 *   機能                 ピン          Nucleo-F303K8   周辺回路
 *   CAN_RX / CAN_TX      PA11 / PA12   D10 / D2        CAN(AF9)→ CAN トランシーバーへ
 *   モーター1 PWM        PA8           D9              TIM1_CH1(AF6)
 *   モーター2 PWM        PA9           D1              TIM1_CH2(AF6)
 *   モーター1 方向       PB0           D3              GPIO 出力
 *   モーター2 方向       PB1           D6              GPIO 出力
 *   エンコーダ1 A / B    PA0 / PA1     A0 / A1         TIM2_CH1 / CH2(AF1)
 *   エンコーダ2 A / B    PB4 / PB5     D12 / D11       TIM3_CH1 / CH2(AF2)
 *   状態 LED             PB3           D13(LD3)        GPIO 出力
 *   USB シリアル TX / RX PA2 / PA15    (ST-LINK の VCP)USART2(AF7)。make LINK=uart のとき通信に使う
 */
#ifndef MAIN_H
#define MAIN_H

#include "stm32f3xx_hal.h"

#define MOTOR1_DIR_Pin GPIO_PIN_0
#define MOTOR1_DIR_GPIO_Port GPIOB
#define MOTOR2_DIR_Pin GPIO_PIN_1
#define MOTOR2_DIR_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB

extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);

#endif /* MAIN_H */
