/*
 * it.c
 */
#include "main.h"

extern CAN_HandleTypeDef hcan;


void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


void CAN_TX_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan);

}

void CAN_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan);

}


void CAN_SCE_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan);
}
