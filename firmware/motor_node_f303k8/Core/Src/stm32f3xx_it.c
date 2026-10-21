#include "stm32f3xx_it.h"

#include "main.h"

void NMI_Handler(void)
{
    while (1) {
    }
}

void HardFault_Handler(void)
{
    Error_Handler();
}

void MemManage_Handler(void)
{
    Error_Handler();
}

void BusFault_Handler(void)
{
    Error_Handler();
}

void UsageFault_Handler(void)
{
    Error_Handler();
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* CAN の受信(FIFO0)→ HAL → can_node.c の HAL_CAN_RxFifo0MsgPendingCallback */
void CAN_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan);
}

/* USB シリアルの受信・送信 → HAL → uart_node.c の HAL_UART_RxCpltCallback など */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/* CAN のエラー・状態変化(バスオフなど)。今は HAL に処理を任せるだけ */
void CAN_SCE_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan);
}
