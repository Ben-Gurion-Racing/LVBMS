
#include "main.h"

void HAL_MspInit(void)
{
//Here will do low level processor specific inits.

	//1. Set up the priority grouping of the arm cortex mx processor
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	//2. Enable the required system exceptions of the arm cortex mx processor
	SCB->SHCSR |= 0x7 << 16; //usage fault, memory fault and bus fault system exceptions

	//3. configure the priority for the system exceptions
	HAL_NVIC_SetPriority(MemoryManagement_IRQn,0,0);
	HAL_NVIC_SetPriority(BusFault_IRQn,0,0);
	HAL_NVIC_SetPriority(UsageFault_IRQn,0,0);

		__HAL_RCC_SYSCFG_CLK_ENABLE();
		__HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(hadc->Instance==ADC1)
	{
		/* Peripheral clock enable */
		__HAL_RCC_ADC12_CLK_ENABLE();
	    __HAL_RCC_GPIOA_CLK_ENABLE();
	    __HAL_RCC_GPIOB_CLK_ENABLE();

	    /**ADC1 GPIO Configuration
	    PA0     ------> ADC1_IN1
	    PA1     ------> ADC1_IN2
	    PA3     ------> ADC1_IN4
	    PB0     ------> ADC1_IN11
	    PB1     ------> ADC1_IN12   */

	    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
	    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	  }

	else if(hadc->Instance==ADC2)
	{
	    /* Peripheral clock enable */
	    __HAL_RCC_ADC12_CLK_ENABLE();
	    __HAL_RCC_GPIOA_CLK_ENABLE();

	    /**ADC2 GPIO Configuration
	    PA4     ------> ADC2_IN1
	    PA5     ------> ADC2_IN2
	    PA7     ------> ADC2_IN4  */

	    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
	    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
	  GPIO_InitTypeDef GPIO_InitStruct;

__HAL_RCC_CAN1_CLK_ENABLE();
__HAL_RCC_GPIOA_CLK_ENABLE();

	/**CAN1 GPIO Configuration
	PA11     ------> CAN1_RX
	PA12     ------> CAN1_TX
	*/

	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(CAN_TX_IRQn,15,0);
	HAL_NVIC_SetPriority(CAN_RX0_IRQn,15,0);
	HAL_NVIC_SetPriority(CAN_RX1_IRQn,15,0);
	HAL_NVIC_SetPriority(CAN_SCE_IRQn,15,0);

	HAL_NVIC_EnableIRQ(CAN_TX_IRQn);
	HAL_NVIC_EnableIRQ(CAN_RX0_IRQn);
	HAL_NVIC_EnableIRQ(CAN_RX1_IRQn);
	HAL_NVIC_EnableIRQ(CAN_SCE_IRQn);

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(huart->Instance==USART2)
	{
	    /* Peripheral clock enable */
	    __HAL_RCC_USART2_CLK_ENABLE();
	    __HAL_RCC_GPIOA_CLK_ENABLE();

	    /**USART2 GPIO Configuration
	    PA2     ------> USART2_TX
	    PA15     ------> USART2_RX  */
	    GPIO_InitStruct.Pin = GPIO_PIN_2|VCP_RX_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
}
